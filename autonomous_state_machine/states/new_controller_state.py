from autonomous_state_machine.states.state_class import State
from autonomous_state_machine.pp import PurePursuit
from autonomous_state_machine.align import AlignAndMove
from geometry_msgs.msg import Twist
import numpy as np
import math

TOLERANCE = 2

class NewControllerState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        with self.shared_data.mutex:
            waypoints = self.shared_data.new_waypoints_odom
            if len(waypoints) < 2:
                self.log("odom waypoints not ready — cannot init PurePursuit", logtype="error")
                self.shared_data.new_waypoints_flag = False
                return

            diff = np.array(waypoints[-1]) - np.array(waypoints[-2])
            next_orientation = math.atan2(diff[1], diff[0])
            self.shared_data.controller = PurePursuit(
                waypoints,
                self.shared_data.cur_heading,
                next_orientation,
                tolerance=TOLERANCE
            )
            self.shared_data.new_waypoints_flag = False
            self.log(f"ENTER — PurePursuit initialized, {len(waypoints)} odom waypoints, goal heading {math.degrees(next_orientation):.1f}°")

    def step(self):
        with self.shared_data.mutex:
            if self.shared_data.controller is None:
                return self.state_instance.new_controller_state
            self.log("PurePursuit ready — → DriveState")
            return self.state_instance.drive_state
