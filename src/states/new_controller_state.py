from state_class import State
from pure_pursuit.pp import PurePursuit
import numpy as np
import math
from std_msgs.msg import Float64MultiArray

class NewControllerState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        print("Entering newControllerState...")
        self.shared_data.mutex.acquire()
        # diff btw the last two waypoints
        # reset_ar_data()
        diff = np.array(self.shared_data.new_waypoints[-1]) - np.array(self.shared_data.new_waypoints[-2])
        next_orientation = math.atan2(diff[1], diff[0])  # calc the angle of the line connecting the last two waypoints
        self.shared_data.controller = PurePursuit(self.shared_data.new_waypoints, 
                                             self.shared_data.cur_heading, 
                                             next_orientation,
                                             tolerance=self.shared_data.TOLERANCE)
        self.shared_data.new_waypoints_flag = False  # invalidate the flag so don't enter again
        self.shared_data.obstacle_avoidance_running = False


        self.shared_data.mutex.release()

    def step(self):
        # # if obstacle then avoid
        # if self.shared_data.new_obstacle_avoidance_waypoints_flag:
        #     return state_instance.new_obstacle_avoidance_controller_state

        self.shared_data.mutex.acquire()

        # Prepare waypoint message for frontend display
        waypoints_msg = Float64MultiArray()
        waypoints_msg.data = []

        for x, y in self.shared_data.controller.waypoints:
            lat, lon = self.shared_data.gps_converter.xy_to_gps(x, y)
            waypoints_msg.data.append(lat)
            waypoints_msg.data.append(lon)

        self.shared_data.auton_path_publisher.publish(waypoints_msg)

        self.shared_data.mutex.release()

        return self.state_instance.drive_state