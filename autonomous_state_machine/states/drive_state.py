from autonomous_state_machine.states.state_class import State, StateInstance
from autonomous_state_machine.shared_data import SharedData
from autonomous_state_machine.align import AlignAndMove
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from time import time
import math
from autonomous_state_machine.utils import clamp

class DriveState(State):
    def __init__(self, shared_data: SharedData, state_instance: StateInstance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        self.log("Entering driveState...")
        self.shared_data.start_time = time()
        led_msg = Float64MultiArray()
        led_msg.data = [0, 0, 255]
        self.shared_data.led_publisher.publish(led_msg)

    def step(self) -> State:
        with self.shared_data.mutex:
            # CRITICAL: If obstacle avoidance is running, DON'T publish commands from DriveState.
            # Let NewObstacleAvoidanceControllerState handle all control.
            if self.shared_data.obstacle_avoidance_running:
                return self.state_instance.drive_state
            
            controller = self.shared_data.controller

            if controller is None or self.shared_data.cur_position is None or self.shared_data.cur_heading is None:
                return self.state_instance.drive_state

            # AlignAndMove publishes Twist internally — let it drive and manage its own target advancement.
            if isinstance(controller, AlignAndMove):
                target_done = controller.step(self.shared_data)
                if target_done:
                    if controller.user_unreached_pts:
                        controller.start_next_target()
                    else:
                        self.log("AlignAndMove: all waypoints reached!")
                        return self.state_instance.validating_state
                if self.shared_data.new_obstacle_avoidance_waypoints_flag:
                    self.log("[DriveState] Flag detected: transitioning to OA state")
                    self.shared_data.obstacle_avoidance_running = True
                    return self.state_instance.new_obstacle_avoidance_controller_state
                if self.shared_data.should_look_for_ar and self.shared_data.found_ar_flag:
                    self.log("AR tag found, switching to go_to_ar_state")
                    self.shared_data.obstacle_avoidance_running = False
                    self.shared_data.new_obstacle_avoidance_waypoints_flag = False
                    return self.state_instance.go_to_ar_state
                if self.shared_data.should_look_for_obj and self.shared_data.found_obj_flag:
                    self.log("Object found, switching to go_to_obj_state")
                    return self.state_instance.go_to_obj_state
                return self.state_instance.drive_state

            # Wrap heading between -pi and pi to avoid large jumps in heading error
            # For example, 179 and -179 are only 2 degrees apart, but pp would give -358 (so a large spin)
            heading = self.shared_data.cur_heading
            heading = math.atan2(math.sin(heading), math.cos(heading))
            effective_speed = clamp(self.shared_data.cur_speed, 0.0, self.shared_data.MAX_AUTON_SPEED)

            # PP was initialized with GPS/NED waypoints (new_waypoints from GPS_2_NED),
            # so pass cur_position (GPS/NED) directly — no odom drift risk.
            cur_pose = [self.shared_data.cur_position[0], self.shared_data.cur_position[1], heading]
            steering_radius = controller.calc_steering(cur_pose)

            # adds steering clamp to prevent small/irrelevant changes from causing a rover spin
            if not steering_radius or abs(steering_radius) < 1e-6:
                steering_output = 0.0
            else:
                # Calculate steering output based on current speed and steering radius - previous code before the clamp
                steering_output = self.shared_data.STEERING_CONST * \
                        effective_speed / steering_radius

            # prevents extremely fast steering, which could also cause o-drive issues -> change this if o-drives keep failing
            max_steer = self.shared_data.AUTON_SPEED * 0.9
            # clamps values to prevent steering output from being unreasonable
            steering_output = clamp(steering_output, -max_steer, max_steer)
            # For obstacle avoidance, keep both wheels moving forward.
            # In-place spin behavior can lead to circling near obstacles.
            if self.shared_data.obstacle_avoidance_running and abs(steering_radius) < 3:
                min_inner_speed = 0.2 * self.shared_data.AUTON_SPEED
                if steering_output > 0:
                    left = max(min_inner_speed, self.shared_data.AUTON_SPEED - abs(steering_output))
                    right = self.shared_data.AUTON_SPEED
                elif steering_output < 0:
                    left = self.shared_data.AUTON_SPEED
                    right = max(min_inner_speed, self.shared_data.AUTON_SPEED - abs(steering_output))
                else:
                    left = self.shared_data.AUTON_SPEED
                    right = self.shared_data.AUTON_SPEED
            else:
                # Normal driving: adjust left/right wheel speeds
                left = self.shared_data.AUTON_SPEED - steering_output
                right = self.shared_data.AUTON_SPEED + steering_output
            # Stop if new obstacle avoidance waypoints are set
            if self.shared_data.new_obstacle_avoidance_waypoints_flag:
                left = 0
                right = 0
                self.log("[DriveState] Flag detected: STOPPING rover (set to [0,0]) before state transition")
            # Clamp wheel speeds and print debug info
            left = clamp(left, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
            right = clamp(right, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
            # self.log(f"heading {self.shared_data.cur_heading} pos {self.shared_data.cur_position} next_pt {controller.waypoints[controller.look_ahead_point_index]}")
            # Publish velocity message to robot as linear/angular twist.
            vel_msg = Twist()
            vel_msg.linear.x = (left + right) / 2.0
            vel_msg.angular.z = (right - left) / 1.53
            # self.log(f"[DriveState] Publishing to auton_publisher: left={left:.3f}, right={right:.3f}")
            self.shared_data.auton_publisher.publish(vel_msg)
            # print(
            #     "CURR POS", round(self.shared_data.cur_position[0], 3), round(self.shared_data.cur_position[1], 3),
            #     "LEFT WHEEL", round(left, 3),
            #     "RIGHT WHEEL", round(right, 3),
            #     "CUR_SPEED", round(self.shared_data.cur_speed, 3),
            # )
            # Check if goal is reached
            if controller.did_reach_goal():
                self.log("Goal reached!")
                return self.state_instance.validating_state
            # Check for AR tag detection
            if self.shared_data.should_look_for_ar and self.shared_data.found_ar_flag:
                self.log("AR tag found, switching to go_to_ar_state")
                # reset obstacle avoidance flags
                self.shared_data.obstacle_avoidance_running = False
                self.shared_data.new_obstacle_avoidance_waypoints_flag = False
                return self.state_instance.go_to_ar_state
            # If new obstacle avoidance waypoints are set, switch state
            if self.shared_data.new_obstacle_avoidance_waypoints_flag:
                self.log("[DriveState] TRANSITIONING to NewObstacleAvoidanceControllerState")
                self.log(f"[DriveState] Waypoints available: {self.shared_data.obstacle_avoidance_new_waypoints}")
                self.shared_data.obstacle_avoidance_running = True
                return self.state_instance.new_obstacle_avoidance_controller_state
            # Check for object detection
            if self.shared_data.should_look_for_obj and self.shared_data.found_obj_flag:
                self.log("Object found, switching to go_to_obj_state")
                return self.state_instance.go_to_obj_state
            return self.state_instance.drive_state
