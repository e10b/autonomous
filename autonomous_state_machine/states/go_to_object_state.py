from autonomous_state_machine.states.state_class import State
from time import time
from std_msgs.msg import Float64MultiArray

OBJ_DRIVE_SPEED = 0.8
OBJ_TIMEOUT = 5

class GoToObjectState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        self.shared_data.counter = 0
        print("Entering goToObjectState...")
        self.shared_data.mutex.acquire()
        self.shared_data.start_time = time()
        self.shared_data.mutex.release()

    def step(self) -> State:
        self.shared_data.mutex.acquire()

        off_center = self.shared_data.obj_cX - 0.5
        print(off_center, self.shared_data.obj_width, self.shared_data.obj_height)
        vel_msg = Float64MultiArray()
        vel_msg.data = [
            OBJ_DRIVE_SPEED + off_center * 1.5,
            OBJ_DRIVE_SPEED - off_center * 1.5,
        ]

        #steering_adjustment = off_center * self.shared_data.STEERING_CONST
        #vel_msg = Float64MultiArray()
        #vel_msg.data = [
        #    self.shared_data.AUTON_SPEED - steering_adjustment,
        #    self.shared_data.AUTON_SPEED + steering_adjustment
        #]

        # Publish velocity commands
        self.shared_data.auton_publisher.publish(vel_msg)  # check if we need different publisher here, ADD

        # if not self.shared_data.go_to_heartbeat and self.shared_data.last_heartbeat - time() > self.shared_data.HEARTBEAT_TIMEOUT:
        #     self.shared_data.mutex.release()
        #     return self.shared_data.heartbeat_state

        # Decide if the object is close enough to stop
        if self.shared_data.obj_width > self.shared_data.OBJ_W_TOLERANCE or self.shared_data.obj_height > self.shared_data.OBJ_H_TOLERANCE :
            vel_msg.data = [0, 0]  # Stop
            self.shared_data.auton_publisher.publish(vel_msg)
            self.shared_data.found_obj_flag = False  # Reset flag
            self.shared_data.mutex.release()
            return self.state_instance.goal_state  # Transition to goal state
        
        if time() - self.shared_data.start_time > OBJ_TIMEOUT:
            self.shared_data.mutex.release()
            return self.state_instance.spin_state

        self.shared_data.mutex.release()
        return self.state_instance.go_to_obj_state