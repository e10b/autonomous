from autonomous_state_machine.states.state_class import State
from time import time

class HeartbeatState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        self.log("ENTER — navigating to last heartbeat position")
        self.shared_data.mutex.acquire()
        self.shared_data.start_time = time()
        self.shared_data.new_waypoints = [self.shared_data.last_heartbeat_pos[-1]]
        self.shared_data.new_waypoints.insert(0, self.shared_data.cur_position)
        self.shared_data.go_to_heartbeat = True
        self.shared_data.mutex.release()
        return self.state_instance.new_controller_state