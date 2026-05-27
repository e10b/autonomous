from autonomous_state_machine.states.state_class import State
from time import time
from geometry_msgs.msg import Twist
from autonomous_state_machine.utils import clamp

AR_DRIVE_SPEED = 1.0

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
            left = clamp(AR_DRIVE_SPEED + off_center * 1.5, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
            right = clamp(AR_DRIVE_SPEED - off_center * 1.5, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
            vel = Twist()
            vel.linear.x = (left + right) / 2.0
            vel.angular.z = (right - left) / 1.53
            self.shared_data.auton_publisher.publish(vel)

            if abs(self.shared_data.ar_size) > self.shared_data.AR_TOLERANCE:
                stop = Twist()
                stop.linear.x = 0.0
                stop.angular.z = 0.0
                self.shared_data.auton_publisher.publish(stop)
                self.shared_data.found_ar_flag = False
                return self.state_instance.goal_state

            if time() - self.shared_data.start_time > self.shared_data.AR_TIMEOUT:
                return self.state_instance.spin_state

            return self.state_instance.go_to_ar_state