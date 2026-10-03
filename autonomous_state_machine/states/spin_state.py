from autonomous_state_machine.states.state_class import State
from time import time
from geometry_msgs.msg import Twist
from autonomous_state_machine.utils import clamp
import autonomous_state_machine.config as config

class SpinState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        self.log("ENTER — spinning to search for AR/object")

        with self.shared_data.mutex:
            self.shared_data.start_time = time()
            self.shared_data.start_heading = self.shared_data.cur_heading

    def step(self) -> State:
        with self.shared_data.mutex:
            self.log(f"spinning {round(time()-self.shared_data.start_time)}s")
            left = clamp(-config.SPIN_SPEED, -config.MAX_AUTON_SPEED, config.MAX_AUTON_SPEED)
            right = clamp(config.SPIN_SPEED, -config.MAX_AUTON_SPEED, config.MAX_AUTON_SPEED)
            vel = Twist()
            vel.linear.x = (left + right) / 2.0
            vel.angular.z = (right - left) / config.TRACK_WIDTH
            self.shared_data.auton_publisher.publish(vel)

            if self.shared_data.found_ar_flag and self.shared_data.should_look_for_ar:
                return self.state_instance.go_to_ar_state

            if self.shared_data.found_obj_flag and self.shared_data.should_look_for_obj:
                return self.state_instance.go_to_obj_state

            if time() - self.shared_data.start_time > 1 and \
                abs(self.shared_data.cur_heading - self.shared_data.start_heading) < config.SPIN_TOLERANCE:  # done with spinning
                return self.state_instance.forward_spiral_state

            return self.state_instance.spin_state