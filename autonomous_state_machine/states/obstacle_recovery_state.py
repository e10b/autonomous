from autonomous_state_machine.states.state_class import State
import numpy as np

class ObstacleRecoveryState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        print("Entering ObstacleRecoveryState...")
        pass

    def step(self):
        controller = self.shared_data.controller
        cur_position = self.shared_data.cur_position

        if controller is None or cur_position is None or not hasattr(controller, "user_pts"):
            self.shared_data.obstacle_avoidance_running = False
            return self.state_instance.drive_state

        user_pts = list(controller.user_pts)
        if len(user_pts) == 0:
            self.shared_data.obstacle_avoidance_running = False
            return self.state_instance.drive_state

        cur = np.array(cur_position)
        closest_idx = int(np.argmin([np.linalg.norm(np.array(pt) - cur) for pt in user_pts]))
        remaining_pts = user_pts[closest_idx:]

        if len(remaining_pts) == 0:
            self.shared_data.obstacle_avoidance_running = False
            return self.state_instance.drive_state

        # Rebuild the main path from current position to remaining user points.
        if np.linalg.norm(np.array(remaining_pts[0]) - cur) < 1e-6:
            self.shared_data.new_waypoints = [cur_position] + remaining_pts[1:]
        else:
            self.shared_data.new_waypoints = [cur_position] + remaining_pts

        self.shared_data.obstacle_avoidance_running = False
        return self.state_instance.new_controller_state

