from state_class import State
from pure_pursuit.pp import PurePursuit
import numpy as np
import math
from std_msgs.msg import Float64MultiArray

TOLERANCE = 2

class NewControllerState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        print("Entering newControllerState...")
        self.shared_data.mutex.acquire()
        # Calculate the difference between the last two waypoints
        diff = np.array(self.shared_data.new_waypoints[-1]) - np.array(self.shared_data.new_waypoints[-2])
        next_orientation = math.atan2(diff[1], diff[0])  # Calculate the angle of the line connecting the last two waypoints
        print(f"Next orientation calculated: {next_orientation}")
        self.shared_data.controller = PurePursuit(
            self.shared_data.new_waypoints,
            self.shared_data.cur_heading,
            next_orientation,
            tolerance=TOLERANCE
        )
        print("PurePursuit controller initialized with new waypoints.")
        self.shared_data.new_waypoints_flag = False  # Invalidate the flag so don't enter again
        self.shared_data.obstacle_avoidance_running = False
        print("Flags updated: new_waypoints_flag set to False, obstacle_avoidance_running set to False.")
        self.shared_data.mutex.release()
        print("Mutex released in on_enter.")

    def step(self):
        # # If obstacle then avoid
        # if self.shared_data.new_obstacle_avoidance_waypoints_flag:
        #     return state_instance.new_obstacle_avoidance_controller_state

        self.shared_data.mutex.acquire()
        print("Mutex acquired in step.")

        # Prepare waypoint message for frontend display
        waypoints_msg = Float64MultiArray()
        waypoints_msg.data = []

        print("Converting waypoints to GPS and preparing message...")
        for x, y in self.shared_data.controller.waypoints:
            lat, lon = self.shared_data.gps_converter.xy_to_gps(x, y)
            waypoints_msg.data.append(lat)
            waypoints_msg.data.append(lon)
            print(f"Waypoint ({x}, {y}) converted to GPS ({lat}, {lon}) and added to message.")

        self.shared_data.auton_path_publisher.publish(waypoints_msg)
        print("Waypoints message published to auton_path_publisher.")

        self.shared_data.mutex.release()
        print("Mutex released in step.")

        return self.state_instance.drive_state