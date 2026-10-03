from autonomous_state_machine.states.state_class import State
from time import time
from geometry_msgs.msg import Twist
from autonomous_state_machine.utils import clamp
import autonomous_state_machine.config as config

class GoToARState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
    
    def on_enter(self):
        self.log("ENTER — visual servo to AR tag (direct velocity control)")

        with self.shared_data.mutex:
            self.shared_data.start_time = time()

    def step(self) -> State:
        with self.shared_data.mutex:

            off_center = self.shared_data.ar_cX - 0.5
            left = clamp(config.GO_TO_AR_DRIVE_SPEED + off_center * config.VISUAL_SERVO_GAIN, -config.MAX_AUTON_SPEED, config.MAX_AUTON_SPEED)
            right = clamp(config.GO_TO_AR_DRIVE_SPEED - off_center * config.VISUAL_SERVO_GAIN, -config.MAX_AUTON_SPEED, config.MAX_AUTON_SPEED)
            vel = Twist()
            vel.linear.x = (left + right) / 2.0
            vel.angular.z = (right - left) / config.TRACK_WIDTH
            self.shared_data.auton_publisher.publish(vel)

            if abs(self.shared_data.ar_size) > config.AR_TOLERANCE:
                stop = Twist()
                stop.linear.x = 0.0
                stop.angular.z = 0.0
                self.shared_data.auton_publisher.publish(stop)
                self.shared_data.found_ar_flag = False
                return self.state_instance.goal_state

            if time() - self.shared_data.start_time > config.AR_TIMEOUT:
                return self.state_instance.spin_state

            return self.state_instance.go_to_ar_state
