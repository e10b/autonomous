from state_class import State
from std_msgs.msg import Float64MultiArray
from time import time
from utils import *
from utils import clamp

class DriveState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
    
    def on_enter(self):
        print("Entering driveState...")  # Notify when entering the drive state
        self.shared_data.start_time = time()

    def step(self):
        self.shared_data.mutex.acquire()
        if self.shared_data.obstacle_avoidance_running:
            controller = self.shared_data.obstacle_avoidance_controller
        else:
            controller = self.shared_data.controller

        # Get current pose (x, y, heading)
        cur_pose = [self.shared_data.cur_position[0],
                    self.shared_data.cur_position[1], self.shared_data.cur_heading]
        steering_radius = controller.calc_steering(cur_pose)
        # Calculate steering output based on current speed and steering radius
        vel_msg = Float64MultiArray()
        steering_output = self.shared_data.STEERING_CONST * \
                self.shared_data.cur_speed / steering_radius
        # For sharp turns in obstacle avoidance, set wheel speeds accordingly
        if self.shared_data.obstacle_avoidance_running and abs(steering_radius) < 3:
            vel_msg.data = [-steering_output/abs(steering_output)*self.shared_data.AUTON_SPEED,
                            steering_output/abs(steering_output)*self.shared_data.AUTON_SPEED]
        else:
            # Normal driving: adjust left/right wheel speeds
            vel_msg.data = [self.shared_data.AUTON_SPEED - steering_output,
                            self.shared_data.AUTON_SPEED + steering_output]
        # Stop if new obstacle avoidance waypoints are set
        if self.shared_data.new_obstacle_avoidance_waypoints_flag:
            vel_msg.data = [0, 0]
        # Clamp wheel speeds and print debug info
        for i in range(len(vel_msg.data)):
            vel_msg.data[i] = clamp(
                vel_msg.data[i], -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
            print("heading", self.shared_data.cur_heading, "pos", self.shared_data.cur_position, 
                "next_pt", controller.waypoints[controller.look_ahead_point_index])  # Print current heading, position, and next waypoint
        # Publish velocity message to robot
        self.shared_data.auton_publisher.publish(vel_msg)
        print(
            "CURR POS", round(self.shared_data.cur_position[0], 3), round(self.shared_data.cur_position[1], 3),
            "FIN POS", round(self.shared_data.new_waypoints[-1][0], 3), round(self.shared_data.new_waypoints[-1][1], 3),
            "LEFT WHEEL", round(vel_msg.data[0], 3),
            "RIGHT WHEEL", round(vel_msg.data[1], 3),
            "CUR_SPEED", round(self.shared_data.cur_speed, 3),
        )
        # Check for heartbeat timeout
        if not self.shared_data.go_to_heartbeat and self.shared_data.last_heartbeat - time() > self.shared_data.HEARTBEAT_TIMEOUT:
            print("Heartbeat timeout detected, switching to heartbeat_state")  # Print heartbeat timeout
            self.shared_data.mutex.release()
            return self.shared_data.heartbeat_state
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