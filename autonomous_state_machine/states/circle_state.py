from autonomous_state_machine.states.state_class import State
from time import time
from geometry_msgs.msg import Twist
from autonomous_state_machine.utils import clamp
import math

class CircleState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
    
    def on_enter(self):
        self.log("ENTER — circling AR tag pole (spin 90° then arc 270°)")

        with self.shared_data.mutex:
            self.shared_data.start_time = time()
            self.shared_data.start_heading = self.shared_data.cur_heading

        self.done_perp = False

    def step(self) -> State:
        with self.shared_data.mutex:
            vel_msg = None

            # Spin 90 degrees CW to be perpendicular to the AR Tag pole
            if not self.done_perp and abs(self.shared_data.cur_heading - (self.shared_data.start_heading - math.pi/2)) > self.shared_data.SPIN_TOLERANCE:
                left = clamp(self.shared_data.SPIN_SPEED, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
                right = clamp(-self.shared_data.SPIN_SPEED, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
                vel = Twist()
                vel.linear.x = (left + right) / 2.0
                vel.angular.z = (right - left) / 1.53
                self.shared_data.auton_publisher.publish(vel)
                return self.state_instance.circle_state
            else:
                self.done_perp = True

            left = clamp(0.5, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
            right = clamp(2, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
            vel2 = Twist()
            vel2.linear.x = (left + right) / 2.0
            vel2.angular.z = (right - left) / 1.53
            self.shared_data.auton_publisher.publish(vel2)

            end_angle = self.shared_data.start_heading + 2/3 * math.pi if self.shared_data.start_heading < 0 \
                        else self.shared_data.start_heading - 2/3 * math.pi
            # Circle 270 degrees to get around the AR Tag pole
            if abs(self.shared_data.cur_heading - end_angle) < self.shared_data.SPIN_TOLERANCE:
                self.shared_data.found_ar_flag = False
                return self.state_instance.goal_state

            return self.state_instance.circle_state