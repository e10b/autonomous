import numpy as np
from autonomous_state_machine.states.state_class import State

class ValidatingState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        print("Entering validatingState...")
        print("Clearing position_history...")
        self.shared_data.mutex.acquire()
        self.shared_data.position_history.clear()
        self.shared_data.mutex.release()

    def step(self):
        # wait for 100 values, then average them
        # check if we are still inside the goal, else set up a new controller
        self.shared_data.mutex.acquire()

        if len(self.shared_data.position_history) >= self.shared_data.HIST_POSE_LEN:
            self.shared_data.cur_position = list(
                np.array(self.shared_data.position_history).mean(axis=0))
            self.shared_data.controller.check_should_redo_goal(
                self.shared_data.cur_position)

            if self.shared_data.should_look_for_ar:
                self.shared_data.mutex.release()
                return self.state_instance.spin_state

                
            if self.shared_data.should_look_for_obj:
                self.shared_data.mutex.release()
                return self.state_instance.spin_state
            
            if not self.shared_data.controller.did_reach_goal():
                self.shared_data.new_waypoints = self.shared_data.new_waypoints[-1:]
                self.shared_data.new_waypoints.insert(0, self.shared_data.cur_position)
                self.shared_data.mutex.release()
                return self.state_instance.new_controller_state
            else:
                self.shared_data.mutex.release()
                return self.state_instance.goal_state

        self.shared_data.mutex.release()
        return self.state_instance.validating_state