from autonomous_state_machine.states.state_class import State
from geometry_msgs.msg import Twist
from time import time
import numpy as np
from autonomous_state_machine.align import AlignAndMove
import autonomous_state_machine.config as config
from autonomous_state_machine.utils import replan_to_goal

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
        self.shared_data.obstacle_avoidance_controller = AlignAndMove(waypoints, logger=self.shared_data.logger)
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

    def _reset_oa_state(self):
        """Clear all obstacle-avoidance bookkeeping so OA can trigger again."""
        self.shared_data.obstacle_avoidance_running = False
        self.shared_data.new_obstacle_avoidance_waypoints_flag = False
        self.shared_data.obstacle_avoidance_new_waypoints = []
        self.shared_data.obstacle_avoidance_controller = None
        self.phase = "idle"
        self.phase_start_time = None

    def _interrupted_spiral(self):
        """Return the spiral state OA interrupted, or None if OA did not interrupt a spiral."""
        spiral_states = (self.state_instance.forward_spiral_state, self.state_instance.backward_spiral_state)
        post_oa = self.shared_data.post_oa_state
        return post_oa if post_oa in spiral_states else None

    def _trim_spiral_to_resume_index(self):
        """Drop the spiral waypoints OA already went around. Returns (resume_idx, remaining_count)."""
        resume_idx = self.shared_data.spiral_resume_index
        self.shared_data.spiral_waypoints = self.shared_data.spiral_waypoints[resume_idx:]
        return resume_idx, len(self.shared_data.spiral_waypoints)

    def _finish_oa(self, completed: bool):
        """Reset OA state and return the state that OA interrupted.

        The caller is responsible for publishing a stop command first.

        Args:
            completed: True if the OA path was followed to the end. The spiral is then
                trimmed to the waypoint where OA rejoins it, and GPS/odom navigation
                re-plans from the new position. False if there was no usable path:
                the spiral resumes where it left off and DriveState resumes as-is.
        """
        self._reset_oa_state()

        spiral = self._interrupted_spiral()
        if spiral is not None:
            if completed:
                resume_idx, remaining = self._trim_spiral_to_resume_index()
                self.log(f"all OA waypoints done — resuming {spiral.__class__.__name__} "
                         f"from waypoint {resume_idx} ({remaining} remaining)")
            else:
                self.log(f"no OA path to follow — resuming {spiral.__class__.__name__}", logtype="warn")
            self.shared_data.spiral_oa_resume = True
            return spiral

        if completed:
            if replan_to_goal(self.shared_data, reason="obstacle avoidance complete"):
                self.log("all OA waypoints done — re-planning to goal → NewControllerState")
                return self.state_instance.new_controller_state
            self.log("all OA waypoints done, path rebuild failed — resuming DriveState", logtype="warn")
        else:
            self.log("no OA path to follow — resuming DriveState", logtype="warn")
        return self.state_instance.drive_state

    def _goal_reached_transition(self):
        """If odom position is within GOAL_REACHED_RADIUS_M of goal_odom, perform cleanup and return
        validating_state. Returns None if the goal has not been reached yet."""
        _cur_pos = self.shared_data.local_odom_position
        _goal    = self.shared_data.goal_odom
        if _cur_pos is None or _goal is None:
            return None
        dist = float(np.linalg.norm(np.array(_cur_pos) - np.array(_goal)))
        if dist >= config.GOAL_REACHED_RADIUS_M:
            return None
        self.log(f"goal reached during AlignAndMove ({dist:.2f}m) — → ValidatingState")
        if self.shared_data.controller is not None:
            self.shared_data.controller.reached_goal = True
        self._reset_oa_state()

        spiral = self._interrupted_spiral()
        if spiral is not None:
            resume_idx, remaining = self._trim_spiral_to_resume_index()
            self.shared_data.spiral_oa_resume = True
            self.log(
                f"OA goal reached while spiraling — resuming spiral instead of validating "
                f"from waypoint {resume_idx} ({remaining} remaining)"
            )
            return spiral

        return self.state_instance.validating_state

    def step(self):
        with self.shared_data.mutex:
            if self.shared_data.new_obstacle_avoidance_waypoints_flag:
                self._load_shared_waypoints()

            if not self.path_waypoints or self.current_waypoint_index >= len(self.path_waypoints):
                self._publish_stop()
                return self._finish_oa(completed=False)

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
                    controller = AlignAndMove(self.path_waypoints, logger=self.shared_data.logger)
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
                    return self._finish_oa(completed=True)
                return self.state_instance.new_obstacle_avoidance_controller_state
