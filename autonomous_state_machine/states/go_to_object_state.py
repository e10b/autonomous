from autonomous_state_machine.states.state_class import State
from time import time
from geometry_msgs.msg import Twist
from autonomous_state_machine.utils import clamp

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
        with self.shared_data.mutex:
            self.shared_data.start_time = time()

    def step(self) -> State:
        with self.shared_data.mutex:
            off_center = self.shared_data.obj_cX - 0.5
            print(off_center, self.shared_data.obj_width, self.shared_data.obj_height)
            left = clamp(OBJ_DRIVE_SPEED + off_center * 1.5, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
            right = clamp(OBJ_DRIVE_SPEED - off_center * 1.5, -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
            vel = Twist()
            vel.linear.x = (left + right) / 2.0
            vel.angular.z = (right - left) / 1.53
            self.shared_data.auton_publisher.publish(vel)  # check if we need different publisher here, ADD

            # if not self.shared_data.go_to_heartbeat and self.shared_data.last_heartbeat - time() > self.shared_data.HEARTBEAT_TIMEOUT:
            #     return self.shared_data.heartbeat_state

            # Decide if the object is close enough to stop
            if self.shared_data.obj_width > self.shared_data.OBJ_W_TOLERANCE or self.shared_data.obj_height > self.shared_data.OBJ_H_TOLERANCE:
                stop = Twist()
                stop.linear.x = 0.0
                stop.angular.z = 0.0
                self.shared_data.auton_publisher.publish(stop)
                self.shared_data.found_obj_flag = False  # Reset flag
                return self.state_instance.goal_state  # Transition to goal state

            if time() - self.shared_data.start_time > OBJ_TIMEOUT:
                return self.state_instance.spin_state

            return self.state_instance.go_to_obj_state