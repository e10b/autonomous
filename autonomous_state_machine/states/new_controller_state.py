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
        self.shared_data.logger.info("[NewControllerState] Entering newControllerState...")
        with self.shared_data.mutex:
            waypoints = self.shared_data.new_waypoints_odom
            if len(waypoints) < 2:
                self.shared_data.logger.error("[NewControllerState] odom waypoints not ready — cannot init controller")
                self.shared_data.new_waypoints_flag = False
                return

            # # --- PurePursuit (commented out for debugging) ---
            # diff = np.array(waypoints[-1]) - np.array(waypoints[-2])
            # next_orientation = math.atan2(diff[1], diff[0])
            # self.shared_data.logger.info(f"[NewControllerState] Next orientation calculated: {next_orientation}")
            # self.shared_data.controller = PurePursuit(
            #     waypoints,
            #     self.shared_data.cur_heading,
            #     next_orientation,
            #     tolerance=TOLERANCE
            # )
            # self.shared_data.logger.info("[NewControllerState] PurePursuit controller initialized with odom waypoints.")

            # --- AlignAndMove (debug replacement for PurePursuit) ---
            self.shared_data.controller = AlignAndMove(waypoints)
            self.shared_data.controller.start_next_target()
            self.shared_data.logger.info("[NewControllerState] AlignAndMove controller initialized with odom waypoints.")

    def step(self):
        # NewControllerState's job is only to initialize the controller and
        # hand off active control to DriveState. DriveState owns publishing
        # and steering logic for both PurePursuit and AlignAndMove.
        with self.shared_data.mutex:
            controller = self.shared_data.controller
            if controller is None:
                return self.state_instance.new_controller_state

            # Ensure controller has an active target (start_next_target called in on_enter).
            return self.state_instance.drive_state
