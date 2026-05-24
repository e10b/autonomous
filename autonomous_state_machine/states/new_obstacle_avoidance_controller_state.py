from autonomous_state_machine.states.state_class import State
from geometry_msgs.msg import Twist
import math
from time import time
import numpy as np
from autonomous_state_machine.align import AlignAndMove
import autonomous_state_machine.config as config

class NewObstacleAvoidanceControllerState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
        
        # Phases = ["reverse", "turn", "forward"] or None if not currently in a recovery maneuver
        self.recovery_phase = None
        
        # Path following
        self.path_waypoints = []  # List of (x, z) tuples in meters
        self.current_waypoint_index = 0
        self.waypoint_threshold = 0.2  # Distance to consider waypoint reached (meters)
        
        # Phase logic: reverse briefly, then follow waypoints
        self.phase = "idle"
        self.phase_start_time = None
        self.reverse_duration = config.BACKUP_DURATION

    def _load_shared_waypoints(self):
        if not self.shared_data.new_obstacle_avoidance_waypoints_flag:
            return False

        waypoints = list(self.shared_data.obstacle_avoidance_new_waypoints)
        if len(waypoints) < 2:
            return False

        self.path_waypoints = waypoints
        self.current_waypoint_index = 1 if len(waypoints) > 1 else 0
        self.shared_data.new_obstacle_avoidance_waypoints_flag = False
        # Create an AlignAndMove controller specifically for obstacle avoidance
        # and start the first target. This controller will publish wheel
        # commands directly to auton_publisher and implement simple rotate-then-move.
        self.shared_data.obstacle_avoidance_controller = AlignAndMove(waypoints)
        self.shared_data.obstacle_avoidance_controller.start_next_target()

        self.phase = "reverse"
        self.phase_start_time = time()
        return True

    def on_enter(self):
        self.shared_data.mutex.acquire()
        try:
            self.shared_data.obstacle_avoidance_controller = None
            self.path_waypoints = []
            self.current_waypoint_index = 0
            self.phase = "idle"
            self.phase_start_time = None
            self._load_shared_waypoints()
            self.shared_data.obstacle_avoidance_running = True
        finally:
            self.shared_data.mutex.release()

    def _publish_stop(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.shared_data.auton_publisher.publish(vel)

    def _publish_reverse(self):
        base_speed = self.shared_data.AUTON_SPEED
        # Convert equal negative wheel speeds into linear/ angular twist
        vel = Twist()
        vel.linear.x = (-base_speed + -base_speed) / 2.0
        vel.angular.z = (-base_speed - (-base_speed)) / 1.53
        self.shared_data.auton_publisher.publish(vel)

    def _rebuild_main_controller_from_current_path(self):
        # Set new_waypoints in map/GPS-NED frame; _convert_waypoints_to_odom() in _tick
        # will TF2-convert them to odom before PurePursuit is re-initialized.
        cur_position = self.shared_data.cur_position
        goal_xy      = self.shared_data.goal_map_xy
        if cur_position is None or goal_xy is None:
            return False
        self.shared_data.new_waypoints = [cur_position, goal_xy]
        self.shared_data.new_waypoints_flag = True
        return True

    def _goal_reached_transition(self):
        """If odom position is within 2m of goal_odom, perform cleanup and return
        validating_state. Returns None if the goal has not been reached yet."""
        _cur_pos = self.shared_data.local_odom_position
        _goal    = self.shared_data.goal_odom
        if _cur_pos is None or _goal is None:
            return None
        if float(np.linalg.norm(np.array(_cur_pos) - np.array(_goal))) >= 2.0:
            return None
        print(f"[OA] Rover at goal ({np.linalg.norm(np.array(_cur_pos)-np.array(_goal)):.2f}m) — entering validating_state")
        if self.shared_data.controller is not None:
            self.shared_data.controller.reached_goal = True
        self.shared_data.obstacle_avoidance_running = False
        self.shared_data.new_obstacle_avoidance_waypoints_flag = False
        self.shared_data.obstacle_avoidance_new_waypoints = []
        self.phase = "idle"
        self.phase_start_time = None
        self.shared_data.obstacle_avoidance_controller = None
        return self.state_instance.validating_state

    def step(self):
        with self.shared_data.mutex:
            if self.shared_data.new_obstacle_avoidance_waypoints_flag:
                self._load_shared_waypoints()

            if not self.path_waypoints:
                self._publish_stop()
                # clear flags to ensure DriveState resumes
                self.shared_data.obstacle_avoidance_running = False
                self.shared_data.new_obstacle_avoidance_waypoints_flag = False
                self.shared_data.obstacle_avoidance_new_waypoints = []
                self.phase = "idle"
                self.phase_start_time = None
                return self.state_instance.drive_state
            
            if self.current_waypoint_index >= len(self.path_waypoints):
                self._publish_stop()
                # clear flags to ensure DriveState resumes
                self.shared_data.obstacle_avoidance_running = False
                self.shared_data.new_obstacle_avoidance_waypoints_flag = False
                self.shared_data.obstacle_avoidance_new_waypoints = []
                self.phase = "idle"
                self.phase_start_time = None
                return self.state_instance.drive_state

            # Phase 1: move backwards briefly to clear the obstacle.
            if self.phase == "reverse":
                if self.phase_start_time is None:
                    self.phase_start_time = time()

                elapsed = time() - self.phase_start_time
                print(f"[OA] REVERSE PHASE: elapsed={elapsed:.2f}s/{self.reverse_duration}s")
                if elapsed < self.reverse_duration:
                    self._publish_reverse()
                    print(f"[OA] Publishing reverse: [-{self.shared_data.AUTON_SPEED:.2f}, -{self.shared_data.AUTON_SPEED:.2f}]")
                    return self.state_instance.new_obstacle_avoidance_controller_state

                print("[OA] Reverse complete, starting waypoint follow")
                self.phase = "follow"
                return self.state_instance.new_obstacle_avoidance_controller_state
            
            # Phase 2: Follow waypoints after reverse using AlignAndMove controller
            if self.phase == "follow":
                # Every-tick goal check — stop immediately if within 2m even mid-path
                result = self._goal_reached_transition()
                if result is not None:
                    return result

                # Ensure we have an AlignAndMove controller
                controller = self.shared_data.obstacle_avoidance_controller
                if controller is None:
                    print("[OA] Retry path picked up: rebuilding AlignAndMove controller")
                    controller = AlignAndMove(self.path_waypoints)
                    controller.start_next_target()
                    self.shared_data.obstacle_avoidance_controller = controller

                target_done = controller.step(self.shared_data)
                if target_done:
                    # If there are more targets, start the next one
                    if controller.user_unreached_pts:
                        controller.start_next_target()
                        return self.state_instance.new_obstacle_avoidance_controller_state
                    # All waypoints done — check goal proximity then rebuild main path
                    print("[OA] ALL WAYPOINTS COMPLETED (AlignAndMove)")
                    self._publish_stop()
                    result = self._goal_reached_transition()
                    if result is not None:
                        return result
                    path_rebuilt = self._rebuild_main_controller_from_current_path()
                    # clear flags and controller so DriveState resumes cleanly
                    self.shared_data.obstacle_avoidance_running = False
                    self.shared_data.new_obstacle_avoidance_waypoints_flag = False
                    self.shared_data.obstacle_avoidance_new_waypoints = []
                    self.phase = "idle"
                    self.phase_start_time = None
                    self.shared_data.obstacle_avoidance_controller = None
                    if path_rebuilt:
                        return self.state_instance.new_controller_state
                    return self.state_instance.drive_state
                return self.state_instance.new_obstacle_avoidance_controller_state
