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
        new_unreached_pts = []
        # remove points we "passed"
        uni = self.shared_data.controller.user_next_index
        last_reached_pt = np.array(self.shared_data.controller.user_pts[uni-1])
        user_unreached_pts = self.shared_data.controller.user_pts[uni:]
        for i, u_pt in enumerate(user_unreached_pts):
            u_pt = np.array(u_pt)
            # if we passed the point, mark it as reached
            if np.dot(u_pt-np.array(self.shared_data.cur_position), u_pt-last_reached_pt) < 0:
                last_reached_pt = u_pt
                continue
            else:
                new_unreached_pts = user_unreached_pts[i:]
                break
        self.shared_data.new_waypoints = new_unreached_pts
        self.shared_data.new_waypoints.insert(0, self.shared_data.cur_position)
        # shared_data.new_waypoints_flag = True
        self.shared_data.obstacle_avoidance_running = False
        return self.state_instance.new_controller_state
