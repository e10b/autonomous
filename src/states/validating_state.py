from state_class import State
import numpy as np

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
        # Wait for enough position samples, then average them and check goal status
        print("ValidatingState: Acquiring mutex to check position history...")
        self.shared_data.mutex.acquire()

        if len(self.shared_data.position_history) >= self.shared_data.HIST_POSE_LEN:
            print(f"Collected {len(self.shared_data.position_history)} positions, averaging...")
            self.shared_data.cur_position = list(
                np.array(self.shared_data.position_history).mean(axis=0))
            print(f"Averaged position: {self.shared_data.cur_position}")
            print("Checking if goal needs to be redone...")
            self.shared_data.controller.check_should_redo_goal(
                self.shared_data.cur_position)

            if self.shared_data.should_look_for_ar:
                print("Should look for AR marker, switching to spin_state.")
                self.shared_data.mutex.release()
                return self.state_instance.spin_state

            if self.shared_data.should_look_for_obj:
                print("Should look for object, switching to spin_state.")
                self.shared_data.mutex.release()
                return self.state_instance.spin_state

            if not self.shared_data.controller.did_reach_goal():
                print("Did not reach goal, setting up new controller state.")
                self.shared_data.new_waypoints = self.shared_data.new_waypoints[-1:]
                self.shared_data.new_waypoints.insert(0, self.shared_data.cur_position)
                self.shared_data.mutex.release()
                return self.state_instance.new_controller_state
            else:
                print("Goal reached, switching to goal_state.")
                self.shared_data.mutex.release()
                return self.state_instance.goal_state

        print("Not enough position samples yet, staying in validatingState.")
        self.shared_data.mutex.release()
        return self.state_instance.validating_state