from autonomous_state_machine.states.state_class import State
from autonomous_state_machine.utils import replan_to_goal

class RetryState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
    
    def on_enter(self):
        self.log("ENTER — spiral search exhausted, returning to goal")

    def step(self) -> State:
        with self.shared_data.mutex:
            # Give up the search: with both flags cleared, ValidatingState sends the
            # rover to GoalState (stopped) on arrival instead of searching again.
            self.shared_data.should_look_for_ar = False
            self.shared_data.should_look_for_obj = False
            if not replan_to_goal(self.shared_data, reason="spiral search exhausted"):
                self.log("cannot return to goal — stopping in StartState", logtype="error")
                return self.state_instance.start_state

        return self.state_instance.new_controller_state
