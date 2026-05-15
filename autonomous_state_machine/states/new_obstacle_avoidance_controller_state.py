from autonomous_state_machine.states.state_class import State
from std_msgs.msg import Float64MultiArray
import math
from time import time
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
        vel_msg = Float64MultiArray()
        vel_msg.data = [0, 0]
        self.shared_data.auton_publisher.publish(vel_msg)

    def _publish_reverse(self):
        base_speed = self.shared_data.AUTON_SPEED
        vel_msg = Float64MultiArray()
        vel_msg.data = [-base_speed, -base_speed]
        self.shared_data.auton_publisher.publish(vel_msg)

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
                # Ensure we have an AlignAndMove controller
                controller = self.shared_data.obstacle_avoidance_controller
                if controller is None:
                    print("[OA] Retry path picked up: rebuilding AlignAndMove controller")
                    controller = AlignAndMove(self.path_waypoints)
                    controller.start_next_target()
                    self.shared_data.obstacle_avoidance_controller = controller

                # Let AlignAndMove publish wheel commands; it returns True when the
                # current target is reached (and advances its internal list).
                target_done = controller.step(self.shared_data)
                if target_done:
                    # If there are more targets, start the next one
                    if controller.user_unreached_pts:
                        controller.start_next_target()
                        return self.state_instance.new_obstacle_avoidance_controller_state
                    # Otherwise finish obstacle avoidance
                    print("[OA] ALL WAYPOINTS COMPLETED (AlignAndMove)")
                    self._publish_stop()
                    # clear flags and controller so DriveState resumes cleanly
                    self.shared_data.obstacle_avoidance_running = False
                    self.shared_data.new_obstacle_avoidance_waypoints_flag = False
                    self.shared_data.obstacle_avoidance_new_waypoints = []
                    self.phase = "idle"
                    self.phase_start_time = None
                    self.shared_data.obstacle_avoidance_controller = None
                    return self.state_instance.drive_state
                return self.state_instance.new_obstacle_avoidance_controller_state
