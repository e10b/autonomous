from autonomous_state_machine.states.state_class import State
from time import time
from std_msgs.msg import Float64MultiArray
from autonomous_state_machine.utils import clamp

AR_DRIVE_SPEED = 1.0

class GoToARState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
    
    def on_enter(self):
        print("Entering goToARState...")

        with self.shared_data.mutex:
            self.shared_data.start_time = time()

    def step(self) -> State:
        with self.shared_data.mutex:
            vel_msg = Float64MultiArray()

            off_center = self.shared_data.ar_cX - 0.5
            print(off_center, self.shared_data.ar_size)
            vel_msg.data = [
                AR_DRIVE_SPEED + off_center * 1.5,
                AR_DRIVE_SPEED - off_center * 1.5,
            ]

            for i in range(len(vel_msg.data)):
                vel_msg.data[i] = clamp(
                    vel_msg.data[i], -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)

            self.shared_data.auton_publisher.publish(vel_msg)

            print(f"LEFT WHEEL: {vel_msg.data[0]} \
                   RIGHT WHEEL: {vel_msg.data[1]}")

            # if the AR tag is big enough in frame, must be close enough

            # if abs(shared_data.ar_size) > shared_data.AR_TOLERANCE and False:
            # if not self.shared_data.go_to_heartbeat and self.shared_data.last_heartbeat - time() > self.shared_data.HEARTBEAT_TIMEOUT:
            #     return self.shared_data.heartbeat_state

            if abs(self.shared_data.ar_size) > self.shared_data.AR_TOLERANCE:
                vel_msg.data = [0, 0]
                self.shared_data.auton_publisher.publish(vel_msg)
                self.shared_data.found_ar_flag = False
                # if self.shared_data.do_gate_task:
                #     self.shared_data.do_gate_task = False
                #     return self.state_instance.circle_state

                return self.state_instance.goal_state

            if time() - self.shared_data.start_time > self.shared_data.AR_TIMEOUT:
                return self.state_instance.spin_state

            return self.state_instance.go_to_ar_state