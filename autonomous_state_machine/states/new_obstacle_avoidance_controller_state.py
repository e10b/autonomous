from autonomous_state_machine.states.state_class import State
from time import time
from std_msgs.msg import Float64MultiArray

class NewObstacleAvoidanceControllerState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
        self.phase_start_time = None
        self.phase_index = 0
        self.maneuver_phases = [
            ("reverse", 2.0),
            ("forward_right", 12.0),
            ("forward_left", 12.0),
            ("forward", 1.5),
        ]

    def on_enter(self):
        print("[NewObstacleAvoidanceControllerState] Entering state...")
        self.shared_data.mutex.acquire()
        try:
            # Invalidate the flag so we don't enter again
            self.shared_data.new_obstacle_avoidance_waypoints_flag = False
            self.shared_data.obstacle_avoidance_controller = None
            self.phase_index = 0
            self.phase_start_time = time()
            self.shared_data.obstacle_avoidance_running = True
            print("[NewObstacleAvoidanceControllerState] Starting fixed OA maneuver.")
        finally:
            self.shared_data.mutex.release()

    def step(self):
        with self.shared_data.mutex:
            if self.phase_start_time is None:
                self.shared_data.obstacle_avoidance_running = False
                return self.state_instance.drive_state

            vel_msg = Float64MultiArray()
            elapsed_time = time() - self.phase_start_time

            while self.phase_index < len(self.maneuver_phases):
                _, phase_duration = self.maneuver_phases[self.phase_index]
                if elapsed_time < phase_duration:
                    break
                elapsed_time -= phase_duration
                self.phase_index += 1
                self.phase_start_time = time() - elapsed_time

            if self.phase_index >= len(self.maneuver_phases):
                vel_msg.data = [0, 0]
                self.shared_data.auton_publisher.publish(vel_msg)
                self.shared_data.obstacle_avoidance_running = False
                self.phase_start_time = None
                return self.state_instance.drive_state

            phase_name, _ = self.maneuver_phases[self.phase_index]
            speed = self.shared_data.AUTON_SPEED
            turn_scale = 0.25

            if phase_name == "reverse":
                vel_msg.data = [-speed, -speed]
            elif phase_name == "forward_right":
                vel_msg.data = [speed, speed * turn_scale]
            elif phase_name == "forward_left":
                vel_msg.data = [speed * turn_scale, speed]
            else:
                vel_msg.data = [speed, speed]

            print(f"[NewObstacleAvoidanceControllerState] Phase={phase_name} elapsed={elapsed_time:.2f}s")
            self.shared_data.auton_publisher.publish(vel_msg)
            return self.state_instance.new_obstacle_avoidance_controller_state
