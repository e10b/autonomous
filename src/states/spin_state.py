from states.state_class import State
import time

class SpinState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        print("Entering SpinState...")

        self.shared_data.mutex.acquire()
        self.shared_data.start_time = time()
        self.shared_data.start_heading = self.shared_data.cur_heading
        self.shared_data.mutex.release()

    def step(self) -> State:
        self.shared_data.mutex.acquire()

        print(f"spinning for {round(time()-self.shared_data.start_time)} seconds")
        vel_msg = self.Float64MultiArray()
        vel_msg.data = [-self.shared_data.SPIN_SPEED, self.shared_data.SPIN_SPEED]
        self.shared_data.auton_publisher.publish(vel_msg)

        if not self.shared_data.go_to_heartbeat and self.shared_data.last_heartbeat - time() > self.shared_data.HEARTBEAT_TIMEOUT:
            self.shared_data.mutex.release()
            return self.shared_data.heartbeat_state

        if self.shared_data.found_ar_flag and self.shared_data.should_look_for_ar:
            self.shared_data.mutex.release()
            return self.state_instance.go_to_ar_state

        if self.shared_data.found_obj_flag and self.shared_data.should_look_for_obj:
            self.shared_data.mutex.release()
            return self.state_instance.go_to_obj_state

        if time() - self.shared_data.start_time > 1 and \
            abs(self.shared_data.cur_heading - self.shared_data.start_heading) < self.shared_data.SPIN_TOLERANCE:  # done with spinning
            self.shared_data.mutex.release()
            return self.state_instance.forward_spiral_state

        self.shared_data.mutex.release()
        return self.state_instance.spin_state