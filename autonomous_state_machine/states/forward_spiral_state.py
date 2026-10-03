from autonomous_state_machine.states.state_class import State
from autonomous_state_machine.spiral_generation import get_spiral
from autonomous_state_machine.pp import PurePursuit
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from time import time
import math
import autonomous_state_machine.config as config

class ForwardSpiralState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        self.shared_data.post_oa_state = self.state_instance.forward_spiral_state
        with self.shared_data.mutex:
            self.shared_data.start_time = time()
            resume = self.shared_data.spiral_oa_resume and len(self.shared_data.spiral_waypoints) >= 2
            if resume:
                self.shared_data.spiral_oa_resume = False
                pts = self.shared_data.spiral_waypoints
                self.log(f"ENTER — resuming outward spiral ({len(pts)} waypoints remaining)")
            else:
                self.shared_data.spiral_oa_resume = False
                if self.shared_data.should_look_for_ar:
                    pts = get_spiral(self.shared_data.cur_position[0], self.shared_data.cur_position[1],
                                        radius=config.SPIRAL_RADIUS, coils=config.SPIRAL_COILS, step=2, direction=1)
                if self.shared_data.should_look_for_obj:
                    pts = get_spiral(self.shared_data.cur_position[0], self.shared_data.cur_position[1],
                                        radius=config.OBJ_SPIRAL_RADIUS, coils=config.OBJ_SPIRAL_COILS, step=2, direction=1)
                self.shared_data.spiral_waypoints = pts
                self.log("ENTER — PurePursuit outward spiral search")
            end_ang = math.atan((pts[-1][1] - pts[-2][1]) /
                                (pts[-1][0] - pts[-2][0]))
            self.shared_data.controller = PurePursuit(pts=pts,
                                                start_ang=self.shared_data.cur_heading,
                                                end_ang=end_ang)
            pub = self.shared_data.spiral_path_publisher
            if pub is not None and len(pts) >= 2:
                node = self.shared_data.node
                stamp = node.get_clock().now().to_msg()
                path_msg = Path()
                path_msg.header.stamp = stamp
                path_msg.header.frame_id = 'odom'
                for (wx, wy) in pts:
                    ps = PoseStamped()
                    ps.header = path_msg.header
                    ps.pose.position.x = float(wx)
                    ps.pose.position.y = float(wy)
                    ps.pose.orientation.w = 1.0
                    path_msg.poses.append(ps)
                pub.publish(path_msg)

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

            # Drop waypoints from the front as the rover passes them so that
            # spiral_waypoints[0] always points to the next unvisited target.
            # OA then resumes ahead instead of backtracking to the start.
            pos = self.shared_data.local_odom_position
            wpts = self.shared_data.spiral_waypoints
            if pos is not None and len(wpts) > 1:
                if math.hypot(pos[0] - wpts[0][0], pos[1] - wpts[0][1]) < config.SPIRAL_WAYPOINT_PRUNE_DIST:
                    wpts.pop(0)

            # print(f"found flag: {self.shared_data.found_ar_flag}, should look: {self.shared_data.should_look_for_ar}")
            if self.shared_data.should_look_for_ar:
                self.log("Searching for target ID:", self.shared_data.target_id)
            elif self.shared_data.should_look_for_obj:
                self.log("Searching for object ID:", self.shared_data.target_obj_id)
            else:
                self.log("Not searching for any object.")

            if self.shared_data.found_ar_flag and self.shared_data.should_look_for_ar:
                self.log("AR tag detected during forward spiral — → GoToARState")
                return self.state_instance.go_to_ar_state

            if self.shared_data.found_obj_flag and self.shared_data.should_look_for_obj:
                self.log("object detected during forward spiral — → GoToObjectState")
                return self.state_instance.goal_state # do not need to go to the object, just see it and stop

            if self.shared_data.controller.did_reach_goal():
                return self.state_instance.backward_spiral_state

            if (self.shared_data.should_look_for_ar and (time() - self.shared_data.start_time > config.SPIRAL_TIMEOUT)):  # Still looking but time's up - switch to backward
                return self.state_instance.backward_spiral_state

            if (self.shared_data.should_look_for_obj and (time() - self.shared_data.start_time > config.OBJ_SPIRAL_TIMEOUT)):  # Still looking but time's up - switch to backward
                return self.state_instance.backward_spiral_state

            return self.state_instance.forward_spiral_state