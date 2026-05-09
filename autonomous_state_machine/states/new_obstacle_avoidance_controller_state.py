from autonomous_state_machine.states.state_class import State
from std_msgs.msg import Float64MultiArray
import math
from time import time

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
        self.reverse_duration = 10.0  # seconds to back up before following the path

    def _load_shared_waypoints(self):
        if not self.shared_data.new_obstacle_avoidance_waypoints_flag:
            return False

        waypoints = list(self.shared_data.obstacle_avoidance_new_waypoints)
        if len(waypoints) < 2:
            return False

        self.path_waypoints = waypoints
        self.current_waypoint_index = 1 if len(waypoints) > 1 else 0
        self.shared_data.new_obstacle_avoidance_waypoints_flag = False
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
                self.shared_data.obstacle_avoidance_running = False
                self.phase = "idle"
                self.phase_start_time = None
                return self.state_instance.drive_state
            
            if self.current_waypoint_index >= len(self.path_waypoints):
                self._publish_stop()
                self.shared_data.obstacle_avoidance_running = False
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
            
            # Phase 2: Follow waypoints after reverse
            if self.phase == "follow":
                if self.current_waypoint_index >= len(self.path_waypoints):
                    print("[OA] All waypoints reached, exiting OA")
                    self._publish_stop()
                    self.shared_data.obstacle_avoidance_running = False
                    self.phase = "idle"
                    self.phase_start_time = None
                    return self.state_instance.drive_state
                
                # Get robot's current position from shared_data
                if self.shared_data.cur_position is None:
                    print("[OA] No position data available")
                    return self.state_instance.new_obstacle_avoidance_controller_state
                
                robot_x, robot_z = self.shared_data.cur_position
                target_x, target_z = self.path_waypoints[self.current_waypoint_index]
                
                # Calculate distance to waypoint
                dx = target_x - robot_x
                dz = target_z - robot_z
                distance = math.sqrt(dx*dx + dz*dz)
                
                print(f"[OA] Waypoint {self.current_waypoint_index}/{len(self.path_waypoints)-1}: pos=({robot_x:.2f},{robot_z:.2f}) target=({target_x:.2f},{target_z:.2f}) dist={distance:.2f}m")
                
                # Check if waypoint reached
                if distance < self.waypoint_threshold:
                    print(f"[OA] Waypoint {self.current_waypoint_index} REACHED")
                    self.current_waypoint_index += 1
                    if self.current_waypoint_index >= len(self.path_waypoints):
                        print("[OA] ALL WAYPOINTS COMPLETED")
                        self._publish_stop()
                        self.shared_data.obstacle_avoidance_running = False
                        self.phase = "idle"
                        self.phase_start_time = None
                        return self.state_instance.drive_state
                    return self.state_instance.new_obstacle_avoidance_controller_state
                
                # Calculate bearing to waypoint (in radians, 0 = forward, positive = left turn)
                bearing = math.atan2(dx, dz)
                
                # Proportional steering: multiply bearing by constant gain (not by speed)
                # This ensures forward motion is always positive
                base_speed = self.shared_data.AUTON_SPEED
                steering_gain = 0.5  # Limit turn radius
                turn_speed = max(-base_speed * 0.5, min(base_speed * 0.5, bearing * steering_gain))
                
                # Publish velocity command (left wheel, right wheel)
                vel_msg = Float64MultiArray()
                left_speed = base_speed + turn_speed
                right_speed = base_speed - turn_speed
                vel_msg.data = [left_speed, right_speed]
                
                print(f"[OA] Publishing: left={left_speed:.2f}, right={right_speed:.2f}, bearing={bearing:.2f}rad")
                self.shared_data.auton_publisher.publish(vel_msg)
                return self.state_instance.new_obstacle_avoidance_controller_state
