from autonomous_state_machine.states.state_class import State
from geometry_msgs.msg import Twist
from time import time
import numpy as np
from autonomous_state_machine.align import AlignAndMove
import autonomous_state_machine.config as config

class NewObstacleAvoidanceControllerState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
        
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
            n = len(self.path_waypoints)
            self.log(f"ENTER — AlignAndMove loaded, {n} waypoints, reversing {self.reverse_duration}s then following path")
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
        vel.angular.z = (-base_speed - (-base_speed)) / self.shared_data.TRACK_WIDTH
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
        dist = float(np.linalg.norm(np.array(_cur_pos) - np.array(_goal)))
        if dist >= 2.0:
            return None
        self.log(f"goal reached during AlignAndMove ({dist:.2f}m) — → ValidatingState")
        if self.shared_data.controller is not None:
            self.shared_data.controller.reached_goal = True
        self.shared_data.obstacle_avoidance_running = False
        self.shared_data.new_obstacle_avoidance_waypoints_flag = False
        self.shared_data.obstacle_avoidance_new_waypoints = []
        self.phase = "idle"
        self.phase_start_time = None
        self.shared_data.obstacle_avoidance_controller = None

        spiral_states = (self.state_instance.forward_spiral_state, self.state_instance.backward_spiral_state)
        if self.shared_data.post_oa_state in spiral_states:
            resume_idx = self.shared_data.spiral_resume_index
            remaining = self.shared_data.spiral_waypoints[resume_idx:]
            self.shared_data.spiral_waypoints = remaining
            self.shared_data.spiral_oa_resume = True
            self.log(
                f"OA goal reached while spiraling — resuming spiral instead of validating "
                f"from waypoint {resume_idx} ({len(remaining)} remaining)"
            )
            return self.shared_data.post_oa_state

        return self.state_instance.validating_state

    def step(self):
        with self.shared_data.mutex:
            if self.shared_data.new_obstacle_avoidance_waypoints_flag:
                self._load_shared_waypoints()

            spiral_states = (self.state_instance.forward_spiral_state, self.state_instance.backward_spiral_state)

            if not self.path_waypoints:
                self._publish_stop()
                self.shared_data.obstacle_avoidance_running = False
                self.shared_data.new_obstacle_avoidance_waypoints_flag = False
                self.shared_data.obstacle_avoidance_new_waypoints = []
                self.phase = "idle"
                self.phase_start_time = None
                post_oa = self.shared_data.post_oa_state
                if post_oa in spiral_states:
                    self.shared_data.spiral_oa_resume = True
                    return post_oa
                return self.state_instance.drive_state

            if self.current_waypoint_index >= len(self.path_waypoints):
                self._publish_stop()
                self.shared_data.obstacle_avoidance_running = False
                self.shared_data.new_obstacle_avoidance_waypoints_flag = False
                self.shared_data.obstacle_avoidance_new_waypoints = []
                self.phase = "idle"
                self.phase_start_time = None
                post_oa = self.shared_data.post_oa_state
                if post_oa in spiral_states:
                    self.shared_data.spiral_oa_resume = True
                    return post_oa
                return self.state_instance.drive_state

            # Phase 1: move backwards briefly to clear the obstacle.
            if self.phase == "reverse":
                if self.phase_start_time is None:
                    self.phase_start_time = time()

                elapsed = time() - self.phase_start_time
                if elapsed < self.reverse_duration:
                    self._publish_reverse()
                    return self.state_instance.new_obstacle_avoidance_controller_state

                self.log(f"reverse complete ({self.reverse_duration:.1f}s) — following {len(self.path_waypoints)} AlignAndMove waypoints")
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
                    self.log("rebuilding AlignAndMove controller for retry path")
                    controller = AlignAndMove(self.path_waypoints)
                    controller.start_next_target()
                    self.shared_data.obstacle_avoidance_controller = controller

                target_done = controller.step(self.shared_data)
                if target_done:
                    if controller.user_unreached_pts:
                        controller.start_next_target()
                        return self.state_instance.new_obstacle_avoidance_controller_state
                    # All waypoints done — check goal proximity, then resume interrupted state
                    self._publish_stop()
                    result = self._goal_reached_transition()
                    if result is not None:
                        return result
                    self.shared_data.obstacle_avoidance_running = False
                    self.shared_data.new_obstacle_avoidance_waypoints_flag = False
                    self.shared_data.obstacle_avoidance_new_waypoints = []
                    self.phase = "idle"
                    self.phase_start_time = None
                    self.shared_data.obstacle_avoidance_controller = None
                    post_oa = self.shared_data.post_oa_state
                    if post_oa in spiral_states:
                        resume_idx = self.shared_data.spiral_resume_index
                        remaining = self.shared_data.spiral_waypoints[resume_idx:]
                        self.shared_data.spiral_waypoints = remaining
                        self.shared_data.spiral_oa_resume = True
                        self.log(f"all OA waypoints done — resuming {post_oa.__class__.__name__} "
                                 f"from waypoint {resume_idx} ({len(remaining)} remaining)")
                        return post_oa
                    path_rebuilt = self._rebuild_main_controller_from_current_path()
                    if path_rebuilt:
                        self.log("all OA waypoints done, goal outside map — rebuilding PurePursuit path → NewControllerState")
                        return self.state_instance.new_controller_state
                    self.log("all OA waypoints done, path rebuild failed — resuming DriveState", logtype="warn")
                    return self.state_instance.drive_state
                return self.state_instance.new_obstacle_avoidance_controller_state
