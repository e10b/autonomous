from autonomous_state_machine.states.state_class import State, StateInstance
from autonomous_state_machine.shared_data import SharedData
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from time import time
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
            if self.shared_data.obstacle_avoidance_running:
                return self.state_instance.drive_state

            controller = self.shared_data.controller
            if controller is None or self.shared_data.cur_position is None or self.shared_data.cur_heading is None:
                return self.state_instance.drive_state

            left, right, done = controller.calc_velocity(self.shared_data)
            if done:
                self.log("Goal reached!")
                return self.state_instance.validating_state
            if left is None:
                return self.state_instance.drive_state

            if self.shared_data.new_obstacle_avoidance_waypoints_flag:
                left, right = 0.0, 0.0
                self.log("[DriveState] Flag detected: STOPPING rover before state transition")

            left = clamp(left, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
            right = clamp(right, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
            vel_msg = Twist()
            vel_msg.linear.x = (left + right) / 2.0
            vel_msg.angular.z = (right - left) / 1.53
            self.shared_data.auton_publisher.publish(vel_msg)

            if self.shared_data.should_look_for_ar and self.shared_data.found_ar_flag:
                self.log("AR tag found, switching to go_to_ar_state")
                self.shared_data.obstacle_avoidance_running = False
                self.shared_data.new_obstacle_avoidance_waypoints_flag = False
                return self.state_instance.go_to_ar_state
            if self.shared_data.new_obstacle_avoidance_waypoints_flag:
                self.log("[DriveState] TRANSITIONING to NewObstacleAvoidanceControllerState")
                self.log(f"[DriveState] Waypoints available: {self.shared_data.obstacle_avoidance_new_waypoints}")
                self.shared_data.obstacle_avoidance_running = True
                return self.state_instance.new_obstacle_avoidance_controller_state
            if self.shared_data.should_look_for_obj and self.shared_data.found_obj_flag:
                self.log("Object found, switching to go_to_obj_state")
                return self.state_instance.go_to_obj_state
            return self.state_instance.drive_state
