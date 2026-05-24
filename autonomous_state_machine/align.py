import math
import numpy as np
from geometry_msgs.msg import Twist
from autonomous_state_machine.utils import clamp


class AlignAndMove():
    """Simple align-and-drive helper that mirrors the controller shape used by
    PurePursuit.

    The class now exposes reusable velocity calculations, while `step()` stays
    as a thin compatibility wrapper that publishes the computed command.
    """

    def __init__(self, pts, start_ang=None, waypoint_threshold: float = 0.2):
        self.user_pts = list(pts)
        self.user_unreached_pts = list(pts)
        self.current_target = None
        self.waypoint_threshold = waypoint_threshold
        self.reached_goal = False

    @property
    def waypoints(self):
        return self.user_pts

    def did_reach_goal(self):
        return self.reached_goal

    def next_point(self, cur_pt):
        # AlignAndMove advances one target at a time, so the active target is the next point.
        return self.current_target

    def calc_velocity(self, shared_data):
        """Return wheel speeds for the current target without publishing them."""
        if self.current_target is None:
            return 0.0, 0.0, True

        if shared_data.local_odom_position is None or shared_data.cur_heading is None:
            return None, None, False

        robot_x, robot_z = shared_data.local_odom_position
        print("Waypoint list:", self.user_unreached_pts)
        print(f"[AlignAndMove] Current position: ({robot_x:.2f}, {robot_z:.2f}), heading: {shared_data.cur_heading:.2f} rad")
        target_x, target_z = self.current_target

        dx = target_x - robot_x
        dz = target_z - robot_z
        distance = math.hypot(dx, dz)

        # If within threshold, advance to next target or signal all done.
        if distance < self.waypoint_threshold:
            print(f"[AlignAndMove] Target reached: {self.current_target} (dist={distance:.2f}m)")
            if self.user_unreached_pts:
                self.user_unreached_pts.pop(0)
            self.current_target = None
            self.phase = None
            if self.user_unreached_pts:
                self.start_next_target()
                return 0.0, 0.0, False
            self.reached_goal = True
            return 0.0, 0.0, True

        # NOTE: obstacle-avoidance waypoints use the same (x,z)
        desired_ang = math.atan2(dz, dx)
        heading = shared_data.cur_heading
        ang_diff = math.atan2(math.sin(desired_ang - heading), math.cos(desired_ang - heading))

        # Align phase: rotate in place until heading error is small.
        ANG_THRESHOLD = 0.05 # 2.8 degrees
        TURN_SPEED = 0.4 * shared_data.AUTON_SPEED

        if self.phase == "align":
            if abs(ang_diff) > ANG_THRESHOLD:
                # Turn in place: left negative, right positive for left turn
                raw_turn = clamp(ang_diff, -1.0, 1.0) * TURN_SPEED
                # Ensure turn command exceeds motor deadzone by enforcing a minimum
                MIN_TURN_SPEED = 0.2 * shared_data.AUTON_SPEED
                if abs(raw_turn) < MIN_TURN_SPEED:
                    enforced = math.copysign(MIN_TURN_SPEED, raw_turn)
                    print(f"[AlignAndMove] Enforcing min turn: raw={raw_turn:.3f} -> enforced={enforced:.3f}")
                    turn = enforced
                else:
                    turn = raw_turn
                left = -turn
                right = turn
                left = clamp(left, -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
                right = clamp(right, -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
                print(f"[AlignAndMove] TURNING: ang_diff={ang_diff:.3f} rad -> left={left:.2f}, right={right:.2f}")
                return left, right, False

            # Aligned, switch to moving forward.
            self.phase = "move"
            print(f"[AlignAndMove] ALIGNED -> switching to MOVE phase (ang_diff={ang_diff:.3f})")
            return 0.0, 0.0, False

        # Move phase: drive straight toward target (no steering).
        left = shared_data.AUTON_SPEED
        right = shared_data.AUTON_SPEED
        left = clamp(left, -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
        right = clamp(right, -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
        print(f"[AlignAndMove] MOVING: dist={distance:.3f}m -> left={left:.2f}, right={right:.2f}")
        return left, right, False

    # def calc_steering(self, cur_pose):
    #     """Return a steering radius similar to PurePursuit.calc_steering(cur_pose).
    #     This lets DriveState treat AlignAndMove like PurePursuit.
    #     """
    #     cur_pt = np.array(cur_pose[:2])
    #     cur_ang = cur_pose[2]

    #     if self.current_target is None:
    #         return float('inf')

    #     target = np.array(self.current_target)
    #     diff = target - cur_pt
    #     d = np.linalg.norm(diff)
    #     if d < 1e-6:
    #         return float('inf')

    #     # Mark reached goal if close to final waypoint
    #     final = np.array(self.user_pts[-1])
    #     if np.linalg.norm(final - cur_pt) < self.waypoint_threshold:
    #         self.reached_goal = True

    #     des_ang = math.atan2(diff[1], diff[0])
    #     ang_diff = math.atan2(math.sin(des_ang - cur_ang), math.cos(des_ang - cur_ang))
    #     sin_ang = math.sin(ang_diff)
    #     if abs(sin_ang) < 1e-6:
    #         return float('inf')

    #     r = d / (2 * sin_ang)
    #     return r

    def check_should_redo_goal(self, cur_pose):
        return not self.reached_goal

    def start_next_target(self):
        """Pop the first unreached point and set it as current target.
        Returns False if there are no remaining targets.
        Sets internal phase to 'align' so the first action is a rotation in place.
        """
        if not self.user_unreached_pts:
            self.current_target = None
            self.phase = None
            self.reached_goal = True
            return False
        self.current_target = tuple(self.user_unreached_pts[0])
        self.phase = "align"
        print(f"[AlignAndMove] New target set: {self.current_target}")
        return True

    def step(self, shared_data) -> bool:
        """Calls calc_velocity, publishes the result, and returns target_done."""
        left, right, target_done = self.calc_velocity(shared_data)
        if left is None:
            return False
        vel = Twist()
        vel.linear.x = (left + right) / 2.0
        vel.angular.z = (right - left) / 1.53
        shared_data.auton_publisher.publish(vel)
        return target_done