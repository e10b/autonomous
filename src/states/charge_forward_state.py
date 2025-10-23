from state_class import State
from std_msgs.msg import Float64MultiArray
from utils import clamp
from time import time

class ChargeForwardState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        print("Entering ChargeForwardState...")

    def step(self) -> State:
        self.shared_data.mutex.acquire()
        vel_msg = Float64MultiArray()
        off_center = (self.shared_data.ar_x_pose + self.shared_data.ar_x_pose2)/2 - 0.5
        vel_msg.data = [
            self.shared_data.AUTON_SPEED / 3 + off_center * self.shared_data.STEERING_CONST * 3,
            self.shared_data.AUTON_SPEED / 3 - off_center * self.shared_data.STEERING_CONST * 3,
        ]
        for i in range(len(vel_msg.data)):
            vel_msg.data[i] = clamp(
                vel_msg.data[i], -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
        self.shared_data.auton_publisher.publish(vel_msg)

        #print(f"LEFT WHEEL: {vel_msg.data[0]} \
        #        RIGHT WHEEL: {vel_msg.data[1]}")

        if self.shared_data.ar_size > 0.01 or\
           self.shared_data.ar_size2 > 0.01:
            self.shared_data.start_time = time()

        print(f"Running for {round(time()-self.shared_data.start_time, 3)} seconds")

        if time() - self.shared_data.start_time > 2:
            vel_msg.data = [0, 0]
            self.shared_data.auton_publisher.publish(vel_msg)
            self.shared_data.found_ar_flag = False
            self.shared_data.mutex.release()
            return self.state_instance.goal_state

        self.shared_data.mutex.release()
        return self.state_instance.charge_forward_state