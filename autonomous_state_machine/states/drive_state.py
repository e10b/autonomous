from autonomous_state_machine.states.state_class import State, StateInstance
from autonomous_state_machine.shared_data import SharedData
from std_msgs.msg import Float64MultiArray
from time import time

class DriveState(State):
    def __init__(self, shared_data: SharedData, state_instance: StateInstance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        ctrl = type(self.shared_data.controller).__name__ if self.shared_data.controller else "None"
        self.log(f"ENTER — controller: {ctrl}, LED blue")
        self.shared_data.post_oa_state = self.state_instance.drive_state
        self.shared_data.start_time = time()
        led_msg = Float64MultiArray()
        led_msg.data = [0, 0, 255]
        self.shared_data.led_publisher.publish(led_msg)

    def step(self) -> State:
        with self.shared_data.mutex:
            if self.shared_data.obstacle_avoidance_running:
                return self.state_instance.drive_state

            controller = self.shared_data.controller
            if controller is None or self.shared_data.cur_position is None or self.shared_data.cur_heading is None:
                return self.state_instance.drive_state

            # step() computes and publishes the Twist for either controller type
            # (PurePursuit or AlignAndMove) and returns True once the goal is reached.
            done = controller.step(self.shared_data)
            if done:
                self.log(f"{type(controller).__name__} reports done — → ValidatingState")
                return self.state_instance.validating_state

            if self.shared_data.should_look_for_ar and self.shared_data.found_ar_flag:
                self.log("AR tag detected — stopping controller, → GoToARState")
                return self.state_instance.go_to_ar_state
            if self.shared_data.should_look_for_obj and self.shared_data.found_obj_flag:
                self.log("object detected — stopping controller, → GoToObjectState")
                return self.state_instance.go_to_obj_state
            return self.state_instance.drive_state
