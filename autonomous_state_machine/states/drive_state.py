from autonomous_state_machine.states.state_class import State
from std_msgs.msg import Float64MultiArray
from time import time
import math
from autonomous_state_machine.utils import clamp

class DriveState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
    
    def on_enter(self):
        print("Entering driveState...")  # Notify when entering the drive state
        self.shared_data.start_time = time()
        led_msg = Float64MultiArray()
        led_msg.data = [0, 0, 255]
        self.shared_data.led_publisher.publish(led_msg)

    def step(self):
        self.shared_data.mutex.acquire()
        if self.shared_data.obstacle_avoidance_running:
            controller = self.shared_data.obstacle_avoidance_controller
            if controller is None:
                self.shared_data.obstacle_avoidance_running = False
                controller = self.shared_data.controller
        else:
            controller = self.shared_data.controller

        if controller is None or self.shared_data.cur_position is None or self.shared_data.cur_heading is None:
            self.shared_data.mutex.release()
            return self.state_instance.drive_state

        # Get current pose (x, y, heading)
        # Wrap heading between -pi and pi to avoid large jumps in heading error
        # For example, 179 and -179 are only 2 degrees apart, but pp would give -358 (so a large spin)
        heading = self.shared_data.cur_heading
        heading = math.atan2(math.sin(heading), math.cos(heading))
        effective_speed = clamp(self.shared_data.cur_speed, 0.0, self.shared_data.MAX_AUTON_SPEED)

        cur_pose = [self.shared_data.cur_position[0],
                    self.shared_data.cur_position[1], heading]
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
        # Clamp wheel speeds and print debug info
        left = clamp(left, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
        right = clamp(right, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
        print("heading", self.shared_data.cur_heading, "pos", self.shared_data.cur_position, 
            "next_pt", controller.waypoints[controller.look_ahead_point_index])  # Print current heading, position, and next waypoint
        # Publish velocity message to robot
        vel_msg = Float64MultiArray()
        vel_msg.data = [left, right]
        self.shared_data.auton_publisher.publish(vel_msg)
        if self.shared_data.new_waypoints and len(self.shared_data.new_waypoints) > 0:
            fin_pos_x = round(self.shared_data.new_waypoints[-1][0], 3)
            fin_pos_y = round(self.shared_data.new_waypoints[-1][1], 3)
        else:
            fin_pos_x = "N/A"
            fin_pos_y = "N/A"
        print(
            "CURR POS", round(self.shared_data.cur_position[0], 3), round(self.shared_data.cur_position[1], 3),
            "FIN POS", fin_pos_x, fin_pos_y,
            "LEFT WHEEL", round(left, 3),
            "RIGHT WHEEL", round(right, 3),
            "CUR_SPEED", round(self.shared_data.cur_speed, 3),
        )
        # Check for heartbeat timeout
        # if not self.shared_data.go_to_heartbeat and self.shared_data.last_heartbeat - time() > self.shared_data.HEARTBEAT_TIMEOUT:
        #     print("Heartbeat timeout detected, switching to heartbeat_state")  # Print heartbeat timeout
        #     self.shared_data.mutex.release()
        #     return self.shared_data.heartbeat_state
        # Check if goal is reached
        if controller.did_reach_goal():
            print("Goal reached!")  # Print when goal is reached
            if self.shared_data.obstacle_avoidance_running:
                self.shared_data.mutex.release()
                return self.state_instance.obstacle_recovery_state
            self.shared_data.mutex.release()
            return self.state_instance.validating_state
        # Check for AR tag detection
        if self.shared_data.should_look_for_ar and self.shared_data.found_ar_flag:
            print("AR tag found, switching to go_to_ar_state")  # Print AR tag found
            # reset obstacle avoidance flags
            self.shared_data.obstacle_avoidance_running = False
            self.shared_data.new_obstacle_avoidance_waypoints_flag = False
            self.shared_data.mutex.release()
            return self.state_instance.go_to_ar_state
        # If new obstacle avoidance waypoints are set, switch state
        if self.shared_data.new_obstacle_avoidance_waypoints_flag:
            print("New obstacle avoidance waypoints detected, switching controller")  # Print new waypoints
            self.shared_data.obstacle_avoidance_running = True
            self.shared_data.mutex.release()
            return self.state_instance.new_obstacle_avoidance_controller_state
        # Check for object detection
        if self.shared_data.should_look_for_obj and self.shared_data.found_obj_flag:
            print("Object found, switching to go_to_obj_state")  # Print object found
            self.shared_data.mutex.release()
            return self.state_instance.go_to_obj_state
        self.shared_data.mutex.release()
        return self.state_instance.drive_state

