from time import time
from autonomous_state_machine.spiral_generation import get_spiral
import math
from autonomous_state_machine.pp import PurePursuit
from geometry_msgs.msg import Twist
from autonomous_state_machine.states.state_class import State

class BackwardSpiralState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        self.shared_data.post_oa_state = self.state_instance.backward_spiral_state
        with self.shared_data.mutex:
            self.shared_data.start_time = time()
            resume = self.shared_data.spiral_oa_resume and len(self.shared_data.spiral_waypoints) >= 2
            if resume:
                self.shared_data.spiral_oa_resume = False
                pts = self.shared_data.spiral_waypoints
                self.log(f"ENTER — resuming inward spiral ({len(pts)} waypoints remaining)")
            else:
                self.shared_data.spiral_oa_resume = False
                if self.shared_data.should_look_for_ar:
                    pts = get_spiral(self.shared_data.cur_position[0], self.shared_data.cur_position[1],
                                        radius=self.shared_data.SPIRAL_RADIUS, coils=self.shared_data.SPIRAL_COILS, step=2, direction=-1)
                if self.shared_data.should_look_for_obj:
                    pts = get_spiral(self.shared_data.cur_position[0], self.shared_data.cur_position[1],
                                        radius=self.shared_data.OBJ_SPIRAL_RADIUS, coils=self.shared_data.OBJ_SPIRAL_COILS, step=2, direction=-1)
                pts = pts[3:]
                self.shared_data.spiral_waypoints = pts
                self.log("ENTER — PurePursuit inward spiral search")
            end_ang = math.atan((pts[-1][1] - pts[-2][1]) /
                                (pts[-1][0] - pts[-2][0]))
            self.shared_data.controller = PurePursuit(pts=pts,
                                                start_ang=self.shared_data.cur_heading,
                                                end_ang=end_ang)

    def step(self) -> State:
        with self.shared_data.mutex:
            if self.shared_data.new_obstacle_avoidance_waypoints_flag:
                stop = Twist()
                stop.linear.x = 0.0
                stop.angular.z = 0.0
                self.shared_data.auton_publisher.publish(stop)
                self.log("obstacle detected during spiral — → OA (AlignAndMove)")
                self.shared_data.obstacle_avoidance_running = True
                return self.state_instance.new_obstacle_avoidance_controller_state
            self.shared_data.controller.step(self.shared_data)

            if self.shared_data.found_ar_flag and self.shared_data.should_look_for_ar:
                self.log("AR tag detected during backward spiral — → GoToARState")
                return self.state_instance.go_to_ar_state

            if self.shared_data.found_obj_flag and self.shared_data.should_look_for_obj:
                self.log("object detected during backward spiral — → GoToObjectState")
                return self.state_instance.goal_state # do not need to go to the object, just see it and stop

            if self.shared_data.controller.did_reach_goal():
                return self.state_instance.backward_spiral_state

            if (self.shared_data.should_look_for_ar and (time() - self.shared_data.start_time > self.shared_data.SPIRAL_TIMEOUT)):  # done with spiraling
                self.log("Timeout reached during backward spiral — → RetryState")
                return self.state_instance.retry_state

            if (self.shared_data.should_look_for_obj and (time() - self.shared_data.start_time > self.shared_data.OBJ_SPIRAL_TIMEOUT)):  # done with spiraling
                self.log("Timeout reached during backward spiral — → RetryState")
                return self.state_instance.retry_state

            return self.state_instance.backward_spiral_state