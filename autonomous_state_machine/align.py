import math
from std_msgs.msg import Float64MultiArray
from autonomous_state_machine.utils import clamp


class AlignAndMove():
    """Simple align-and-drive helper that publishes wheel commands to the
    same `auton_publisher` used by DriveState. The API is intentionally
    minimal: call `start_next_target()` to set the next target, then call
    `step(shared_data)` repeatedly from the main loop. `step` will publish
    wheel speeds and return True when the current target is reached.

    This mirrors DriveState's steering -> wheel-speed conversion so behavior
    is consistent with the rest of the controller stack.
    """

    def __init__(self, pts, start_ang=None, waypoint_threshold: float = 0.2):
        self.user_pts = list(pts)
        self.user_unreached_pts = list(pts)
        self.current_target = None
        self.waypoint_threshold = waypoint_threshold

    def start_next_target(self):
        """Pop the first unreached point and set it as current target.
        Returns False if there are no remaining targets.
        Sets internal phase to 'align' so the first action is a rotation in place.
        """
        if not self.user_unreached_pts:
            self.current_target = None
            self.phase = None
            return False
        self.current_target = tuple(self.user_unreached_pts[0])
        self.phase = "align"
        print(f"[AlignAndMove] New target set: {self.current_target}")
        return True

    def _publish_stop(self, shared_data):
        vel_msg = Float64MultiArray()
        vel_msg.data = [0.0, 0.0]
        shared_data.auton_publisher.publish(vel_msg)

    def step(self, shared_data) -> bool:
        """Call repeatedly. Publishes wheel commands towards `current_target`.
        Returns True when the current target has been reached (and advances
        the internal list), False otherwise.
        """
        if self.current_target is None:
            # No active target
            return True

        # Need position and heading
        if shared_data.local_odom_position is None or shared_data.cur_heading is None:
            return False

        robot_x, robot_z = shared_data.local_odom_position
        print("Waypoint list:", self.user_unreached_pts)
        print(f"[AlignAndMove] Current position: ({robot_x:.2f}, {robot_z:.2f}), heading: {shared_data.cur_heading:.2f} rad")
        target_x, target_z = self.current_target

        dx = target_x - robot_x
        dz = target_z - robot_z
        distance = math.hypot(dx, dz)

        # If within threshold, mark reached and advance
        if distance < self.waypoint_threshold:
            print(f"[AlignAndMove] Target reached: {self.current_target} (dist={distance:.2f}m)")
            self.user_unreached_pts.pop(0)
            self._publish_stop(shared_data)
            self.current_target = None
            self.phase = None
            return True

        # NOTE: obstacle-avoidance waypoints use the same (x,z)
        desired_ang = math.atan2(dz, dx)
        # can make this -shared_data.cur_heading if we want to test reverse
        heading = shared_data.cur_heading
        # heading = shared_data.cur_heading
        ang_diff = math.atan2(math.sin(desired_ang - heading), math.cos(desired_ang - heading))

        # Align phase: rotate in place until heading error is small
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
                vel_msg = Float64MultiArray()
                vel_msg.data = [left, right]
                shared_data.auton_publisher.publish(vel_msg)
                print(f"[AlignAndMove] TURNING: ang_diff={ang_diff:.3f} rad -> left={left:.2f}, right={right:.2f}")
                return False
            else:
                # Aligned, switch to moving forward
                self.phase = "move"
                print(f"[AlignAndMove] ALIGNED -> switching to MOVE phase (ang_diff={ang_diff:.3f})")
                return False

        # Move phase: drive straight toward target (no steering)
        left = shared_data.AUTON_SPEED
        right = shared_data.AUTON_SPEED
        left = clamp(left, -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
        right = clamp(right, -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
        vel_msg = Float64MultiArray()
        vel_msg.data = [left, right]
        shared_data.auton_publisher.publish(vel_msg)
        print(f"[AlignAndMove] MOVING: dist={distance:.3f}m -> left={left:.2f}, right={right:.2f}")
        return False
        