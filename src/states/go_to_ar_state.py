from state_class import State
import time
from std_msgs.msg import Float64MultiArray
from utils import clamp

class GoToARState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
    
    def on_enter(self):
        print("Entering goToARState...")

        self.shared_data.mutex.acquire()
        self.shared_data.start_time = time()
        self.shared_data.mutex.release()

    def step(self) -> State:
        self.shared_data.mutex.acquire()

        vel_msg = Float64MultiArray()

        off_center = self.shared_data.ar_cX - 0.5
        print(off_center, self.shared_data.ar_size)
        vel_msg.data = [
            self.shared_data.AR_DRIVE_SPEED + off_center * 1.5,
            self.shared_data.AR_DRIVE_SPEED - off_center * 1.5,
        ]

        for i in range(len(vel_msg.data)):
            vel_msg.data[i] = clamp(
                vel_msg.data[i], -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
        self.shared_data.auton_publisher.publish(vel_msg)

        # print(f"LEFT WHEEL: {vel_msg.data[0]} \
        #        RIGHT WHEEL: {vel_msg.data[1]}")

        # if the AR tag is big enough in frame, must be close enough

        # if abs(shared_data.ar_size) > shared_data.AR_TOLERANCE and False:
        if not self.shared_data.go_to_heartbeat and self.shared_data.last_heartbeat - time() > self.shared_data.HEARTBEAT_TIMEOUT:
            self.shared_data.mutex.release()
            return self.shared_data.heartbeat_state

        if abs(self.shared_data.ar_size) > self.shared_data.AR_TOLERANCE:
            vel_msg.data = [0, 0]
            self.shared_data.auton_publisher.publish(vel_msg)
            self.shared_data.found_ar_flag = False
            if self.shared_data.do_gate_task:
                self.shared_data.do_gate_task = False
                self.shared_data.mutex.release()
                return self.state_instance.circle_state

            self.shared_data.mutex.release()
            return self.state_instance.goal_state

        if time() - self.shared_data.start_time > self.shared_data.AR_TIMEOUT:
            self.shared_data.mutex.release()
            return self.state_instance.spin_state

        self.shared_data.mutex.release()
        return self.state_instance.go_to_ar_state