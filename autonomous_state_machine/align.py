import math
import numpy as np


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

    def calc_steering(self, cur_pose):
        """Return a steering radius similar to PurePursuit.calc_steering(cur_pose).
        This lets DriveState treat AlignAndMove like PurePursuit.
        """
        cur_pt = np.array(cur_pose[:2])
        cur_ang = cur_pose[2]

        if self.current_target is None:
            return float('inf')

        target = np.array(self.current_target)
        diff = target - cur_pt
        d = np.linalg.norm(diff)
        if d < 1e-6:
            return float('inf')

        # Mark reached goal if close to final waypoint
        final = np.array(self.user_pts[-1])
        if np.linalg.norm(final - cur_pt) < self.waypoint_threshold:
            self.reached_goal = True

        des_ang = math.atan2(diff[1], diff[0])
        ang_diff = math.atan2(math.sin(des_ang - cur_ang), math.cos(des_ang - cur_ang))
        sin_ang = math.sin(ang_diff)
        if abs(sin_ang) < 1e-6:
            return float('inf')

        r = d / (2 * sin_ang)
        return r

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