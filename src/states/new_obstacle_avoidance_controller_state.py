from state_class import State
import math
from time import time
from numpy import np
from pure_pursuit.pp import PurePursuit
from std_msgs.msg import Float64MultiArray

class NewObstacleAvoidanceControllerState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        print("Entering NewObstacleAvoidanceControllerState...")
        self.shared_data.mutex.acquire()
        # Invalidate the flag so we don't enter again
        self.shared_data.new_obstacle_avoidance_waypoints_flag = False

        # Calculate the difference between the last two waypoints
        diff = np.array(self.shared_data.obstacle_avoidance_new_waypoints[-1]) - np.array(
            self.shared_data.obstacle_avoidance_new_waypoints[-2])
        print(f"Waypoint difference: {diff}")

        # Calculate the angle of the line connecting the last two waypoints
        next_orientation = math.atan2(diff[1], diff[0])
        print(f"Next orientation (radians): {next_orientation}")

        # Initialize the PurePursuit controller with the new waypoints and orientation
        self.shared_data.obstacle_avoidance_controller = PurePursuit(
            self.shared_data.obstacle_avoidance_new_waypoints,
            self.shared_data.cur_heading,
            next_orientation,
            fineness=0.1,
            max_look_ahead=0.5
        )
        self.shared_data.obstacle_avoidance_controller.waypoints = np.array(self.shared_data.obstacle_avoidance_new_waypoints)
        self.shared_data.start_time = time()
        print("PurePursuit controller initialized.")
        self.shared_data.mutex.release()

    def step(self):
        # If obstacle, then avoid
        self.shared_data.mutex.acquire()
        vel_msg = Float64MultiArray()
        elapsed_time = time() - self.shared_data.start_time
        print(f"Elapsed time since start: {elapsed_time:.2f} seconds")
        if elapsed_time < 2:
            print("Reversing to avoid obstacle.")
            print(f"Current position: {self.shared_data.cur_position}")
            vel_msg.data = [-self.shared_data.AUTON_SPEED,
                            -self.shared_data.AUTON_SPEED]
            self.shared_data.auton_publisher.publish(vel_msg)
            self.shared_data.mutex.release()
            return self.state_instance.new_obstacle_avoidance_controller_state
        print("Stopping after obstacle avoidance maneuver.")
        vel_msg.data = [0, 0]
        self.shared_data.auton_publisher.publish(vel_msg)
        self.shared_data.mutex.release()
        return self.state_instance.drive_state