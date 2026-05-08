from autonomous_state_machine.states.state_class import State

class RetryState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
    
    def on_enter(self):
        print("Entering retryState...")

    def step(self) -> State:
        with self.shared_data.mutex:
            self.shared_data.new_waypoints = [self.shared_data.go_home_position]
            self.shared_data.new_waypoints.insert(0, self.shared_data.cur_position)
            self.shared_data.should_look_for_ar = False

        return self.state_instance.new_controller_state
