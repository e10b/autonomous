from autonomous_state_machine.states.state_class import State
import time
from std_msgs.msg import Float64MultiArray
import math

class CircleState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
    
    def on_enter(self):
        print("Entering CircleState...")

        self.shared_data.mutex.acquire()
        self.shared_data.start_time = time()
        self.shared_data.start_heading = self.shared_data.cur_heading
        self.shared_data.mutex.release()

        self.done_perp = False

    def step(self) -> State:
        self.shared_data.mutex.acquire()
        vel_msg = Float64MultiArray()

        # Spin 90 degrees CW to be perpendicular to the AR Tag pole
        if not self.done_perp and abs(self.shared_data.cur_heading - (self.shared_data.start_heading - math.pi/2)) > self.shared_data.SPIN_TOLERANCE:
            print(f"Spinning 90 deg for {round(time()-self.shared_data.start_time, 3)} seconds { abs(self.shared_data.cur_heading - (self.shared_data.start_heading - math.pi/2))}")
            """if time() - shared_data.start_time < shared_data.SPIN_TIMEOUT/4:
            print(
                f"Spinning 90 deg for {round(time()-shared_data.start_time, 3)} seconds")"""
            vel_msg.data = [self.shared_data.SPIN_SPEED, -self.shared_data.SPIN_SPEED]
            self.shared_data.auton_publisher.publish(vel_msg)
            self.shared_data.mutex.release()
            return self.state_instance.circle_state
        else:
            self.done_perp = True
        print(f"Spinning 270 deg for {round(time()-self.shared_data.start_time, 3)} seconds")

        vel_msg.data = [
            0.5,
            2,
        ]
        self.shared_data.auton_publisher.publish(vel_msg)

        end_angle = self.shared_data.start_heading + 2/3 * math.pi if self.shared_data.start_heading < 0 \
                    else self.shared_data.start_heading - 2/3 * math.pi
        # Circle 270 degrees to get around the AR Tag pole
        if abs(self.shared_data.cur_heading - end_angle) < self.shared_data.SPIN_TOLERANCE:
            self.shared_data.found_ar_flag = False
            self.shared_data.mutex.release()
            return self.state_instance.goal_state

        self.shared_data.mutex.release()
        return self.state_instance.circle_state