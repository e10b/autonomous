from autonomous_state_machine.states.state_class import State
from autonomous_state_machine.pp import PurePursuit
from autonomous_state_machine.align import AlignAndMove
import autonomous_state_machine.config as config
import numpy as np
import math

class NewControllerState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        with self.shared_data.mutex:
            waypoints = self.shared_data.new_waypoints_odom
            if len(waypoints) < 2:
                self.log("odom waypoints not ready — cannot init controller", logtype="error")
                self.shared_data.new_waypoints_flag = False
                return

            # Check if global terrain/rock obstacles require global Theta* path generation
            gm = getattr(self.shared_data, "global_map", None)
            if gm is not None and getattr(gm, "static_slope_data", None) is not None:
                start_pt = waypoints[0]
                goal_pt = waypoints[-1]
                planned = gm.get_path(
                    start_pt[0], start_pt[1],
                    goal_pt[0], goal_pt[1],
                    robot_radius_m=0.35
                )
                if len(planned) < 2:
                    planned = gm.get_path(
                        start_pt[0], start_pt[1],
                        goal_pt[0], goal_pt[1],
                        robot_radius_m=0.28
                    )
                if len(planned) >= 2:
                    self.log(f"[GlobalMap] Injected global Theta* trajectory ({len(planned)} waypoints) around static slope & rock obstacles")
                    waypoints = planned

            if config.CONTROLLER_TYPE == "align":
                self.shared_data.controller = AlignAndMove(waypoints, logger=self.shared_data.logger)
                self.shared_data.controller.start_next_target()
                self.log(f"ENTER — AlignAndMove initialized, {len(waypoints)} odom waypoints")
            else:
                diff = np.array(waypoints[-1]) - np.array(waypoints[-2])
                next_orientation = math.atan2(diff[1], diff[0])
                self.shared_data.controller = PurePursuit(
                    waypoints,
                    self.shared_data.cur_heading,
                    next_orientation,
                    tolerance=config.GOAL_REACHED_RADIUS_M
                )
                self.log(f"ENTER — PurePursuit initialized, {len(waypoints)} odom waypoints, goal heading {math.degrees(next_orientation):.1f}°")
            self.shared_data.new_waypoints_flag = False

    def step(self):
        with self.shared_data.mutex:
            if self.shared_data.controller is None:
                return self.state_instance.new_controller_state
            self.log(f"{type(self.shared_data.controller).__name__} ready — → DriveState")
            return self.state_instance.drive_state
