from autonomous_state_machine.states.state_class import State
from autonomous_state_machine.pp import PurePursuit
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
        self.shared_data.logger.info("[NewControllerState] Entering newControllerState...")
        with self.shared_data.mutex:
            waypoints = self.shared_data.new_waypoints_odom
            if len(waypoints) < 2:
                self.shared_data.logger.error("[NewControllerState] odom waypoints not ready — cannot init PP")
                self.shared_data.new_waypoints_flag = False
                return
            diff = np.array(waypoints[-1]) - np.array(waypoints[-2])
            next_orientation = math.atan2(diff[1], diff[0])
            self.shared_data.logger.info(f"[NewControllerState] Next orientation calculated: {next_orientation}")
            self.shared_data.controller = PurePursuit(
                waypoints,
                self.shared_data.cur_heading,
                next_orientation,
                tolerance=TOLERANCE
            )
            self.shared_data.logger.info("[NewControllerState] PurePursuit controller initialized with odom waypoints.")
            self.shared_data.new_waypoints_flag = False
            self.shared_data.obstacle_avoidance_running = False
            self.shared_data.logger.info("[NewControllerState] Flags updated.")

    def step(self):
        with self.shared_data.mutex:
            # PP waypoints are in odom frame; convert to map (GPS/NED) frame before
            # calling xy_to_gps for visualization. In sim map≡odom so offset=(0,0).
            offset = self.shared_data.gps_ned_to_map_offset or (0.0, 0.0)
            waypoints_msg = Float64MultiArray()
            waypoints_msg.data = []
            for x, y in self.shared_data.controller.waypoints:
                lat, lon = self.shared_data.gps_converter.xy_to_gps(x + offset[0], y + offset[1])
                waypoints_msg.data.append(lat)
                waypoints_msg.data.append(lon)
            self.shared_data.auton_path_publisher.publish(waypoints_msg)

        return self.state_instance.drive_state
