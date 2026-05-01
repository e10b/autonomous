from time import time
from autonomous_state_machine.spiral_generation import get_spiral
import math
from autonomous_state_machine.pp import PurePursuit
from autonomous_state_machine.states.state_class import State
from std_msgs.msg import Float64MultiArray
from autonomous_state_machine.utils import clamp

class BackwardSpiralState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        print("Entering BackwardSpiralState...")
        self.shared_data.mutex.acquire()
        self.shared_data.start_time = time()
        if self.shared_data.should_look_for_ar:
            pts = get_spiral(self.shared_data.cur_position[0], self.shared_data.cur_position[1], 
                                radius=self.shared_data.SPIRAL_RADIUS, coils=self.shared_data.SPIRAL_COILS, step=2, direction=-1)
        if self.shared_data.should_look_for_obj:
            pts = get_spiral(self.shared_data.cur_position[0], self.shared_data.cur_position[1], 
                                radius=self.shared_data.OBJ_SPIRAL_RADIUS, coils=self.shared_data.OBJ_SPIRAL_COILS, step=2, direction=-1)
        pts = pts[3:]
        end_ang = math.atan((pts[-1][1] - pts[-2][1]) /
                            (pts[-1][0] - pts[-2][0]))
        self.shared_data.controller = PurePursuit(pts=pts, 
                                            start_ang=self.shared_data.cur_heading, 
                                            end_ang=end_ang)
        self.shared_data.mutex.release()

    def step(self) -> State:
        self.shared_data.mutex.acquire()
        cur_pose = [self.shared_data.cur_position[0], self.shared_data.cur_position[1], self.shared_data.cur_heading]
        steering_radius = self.shared_data.controller.calc_steering(cur_pose)
        steering_output = self.shared_data.STEERING_CONST * self.shared_data.AUTON_SPEED / steering_radius / 2
        vel_msg = Float64MultiArray()
        vel_msg.data = [self.shared_data.AUTON_SPEED - steering_output,
                        self.shared_data.AUTON_SPEED + steering_output]
        for i in range(len(vel_msg.data)):
            vel_msg.data[i] = clamp(vel_msg.data[i], -self.shared_data.MAX_AUTON_SPEED, self.shared_data.MAX_AUTON_SPEED)
        self.shared_data.auton_publisher.publish(vel_msg)

        # print(
        #      "CURR POS", round(shared_data.cur_position[0], 3), round(shared_data.cur_position[1], 3),
        #      "FIN POS", round(shared_data.new_waypoints[-1][0], 3), round(shared_data.new_waypoints[-1][1], 3),
        #      "LEFT WHEEL", round(vel_msg.data[0], 3),
        #      "RIGHT WHEEL", round(vel_msg.data[1], 3),
        # )
        # if not self.shared_data.go_to_heartbeat and self.shared_data.last_heartbeat - time() > self.shared_data.HEARTBEAT_TIMEOUT:
        #     self.shared_data.mutex.release()
        #     return self.shared_data.heartbeat_state

        print(f"found flag: {self.shared_data.found_ar_flag}, should look: {self.shared_data.should_look_for_ar}")

        if self.shared_data.found_ar_flag and self.shared_data.should_look_for_ar:
            self.shared_data.mutex.release()
            print("Going to AR state!")
            return self.state_instance.go_to_ar_state
        
        if self.shared_data.found_obj_flag and self.shared_data.should_look_for_obj:
            self.shared_data.mutex.release()
            print("Going to object state!")
            return self.state_instance.go_to_obj_state

        if self.shared_data.controller.did_reach_goal():
            self.shared_data.mutex.release()
            return self.state_instance.backward_spiral_state

        if (self.shared_data.should_look_for_ar and (time() - self.shared_data.start_time > self.shared_data.SPIRAL_TIMEOUT)):  # done with spiraling
        # if shared_data.controller.did_reach_goal() or \
        #    time() - shared_data.start_time > shared_data.AR_TIMEOUT:
            self.shared_data.mutex.release()
            return self.state_instance.retry_state

        if (self.shared_data.should_look_for_obj and (time() - self.shared_data.start_time > self.shared_data.OBJ_SPIRAL_TIMEOUT)):  # done with spiraling
        # if shared_data.controller.did_reach_goal() or \
        #    time() - shared_data.start_time > shared_data.AR_TIMEOUT:
            self.shared_data.mutex.release()
            return self.state_instance.retry_state
        

        self.shared_data.mutex.release()
        return self.state_instance.backward_spiral_state