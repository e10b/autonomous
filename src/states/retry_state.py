from state_class import State
class RetryState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
    
    def on_enter(self):
        print("Entering retryState...")

    def step(self) -> State:
        self.shared_data.mutex.acquire()
        self.shared_data.new_waypoints = [self.shared_data.go_home_position]
        self.shared_data.new_waypoints.insert(0, self.shared_data.cur_position)
        self.shared_data.should_look_for_ar = False

        self.on_entershared_data.mutex.release()
        return self.state_instance.new_controller_state
