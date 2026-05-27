from autonomous_state_machine.states.state_class import State
from autonomous_state_machine.spiral_generation import get_spiral
from autonomous_state_machine.pp import PurePursuit
from time import time
import math

class ForwardSpiralState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        print("Entering ForwardSpiralState...")
        with self.shared_data.mutex:
            self.shared_data.start_time = time()
            if self.shared_data.should_look_for_ar:
                pts = get_spiral(self.shared_data.cur_position[0], self.shared_data.cur_position[1],
                                    radius=self.shared_data.SPIRAL_RADIUS, coils=self.shared_data.SPIRAL_COILS, step=2, direction=1)
            if self.shared_data.should_look_for_obj:
                pts = get_spiral(self.shared_data.cur_position[0], self.shared_data.cur_position[1],
                                    radius=self.shared_data.OBJ_SPIRAL_RADIUS, coils=self.shared_data.OBJ_SPIRAL_COILS, step=2, direction=1)
            end_ang = math.atan((pts[-1][1] - pts[-2][1]) /
                                (pts[-1][0] - pts[-2][0]))
            self.shared_data.controller = PurePursuit(pts=pts,
                                                start_ang=self.shared_data.cur_heading,
                                                end_ang=end_ang)

    def step(self) -> State:
        with self.shared_data.mutex:
            self.shared_data.controller.step(self.shared_data)

            # print(f"found flag: {self.shared_data.found_ar_flag}, should look: {self.shared_data.should_look_for_ar}")
            if self.shared_data.should_look_for_ar:
                print("Searching for target ID:", self.shared_data.target_id)
            elif self.shared_data.should_look_for_obj:
                print("Searching for object ID:", self.shared_data.target_obj_id)
            else:
                print("Not searching for any object.")

            if self.shared_data.found_ar_flag and self.shared_data.should_look_for_ar:
                print("Going to AR state!")
                return self.state_instance.go_to_ar_state

            if self.shared_data.found_obj_flag and self.shared_data.should_look_for_obj:
                print("Going to object state!")
                return self.state_instance.go_to_obj_state

            if self.shared_data.controller.did_reach_goal():
                return self.state_instance.backward_spiral_state

            if (self.shared_data.should_look_for_ar and (time() - self.shared_data.start_time > self.shared_data.SPIRAL_TIMEOUT)):  # done with spiraling
                return self.state_instance.backward_spiral_state

            if (self.shared_data.should_look_for_obj and (time() - self.shared_data.start_time > self.shared_data.OBJ_SPIRAL_TIMEOUT)):  # done with spiraling
                return self.state_instance.backward_spiral_state

            return self.state_instance.forward_spiral_state