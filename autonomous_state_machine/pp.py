import math
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.integrate import quad
from scipy.spatial import KDTree
from geometry_msgs.msg import Twist
from autonomous_state_machine.utils import clamp


class PurePursuit():
  def __init__(self, pts, start_ang, end_ang, tolerance=1.5, URC_requirement=3, min_look_ahead=1/2, max_look_ahead=2, fineness=0.01):
    self.user_pts = list(pts)
    self.user_unreached_pts = list(pts)
    self.final = np.array(pts[-1])
    self.path = self.generate_path(pts, [start_ang, end_ang])
    self.waypoints = self.get_waypoints(self.path, fineness)
    self.num_waypoints = len(self.waypoints)
    self.tree = self.build_waypoint_tree(self.waypoints)

    self.tolerance = tolerance
    self.URC_requirement = URC_requirement
    self.min_look_ahead = min_look_ahead
    self.max_look_ahead = max_look_ahead

    self.look_ahead_point_index = 0
    self.look_ahead_distance = max_look_ahead

    self.reached_goal = False
    self.outside_lookahead = False

  def did_reach_goal(self):
    return self.reached_goal

  def calc_velocity(self, shared_data):
    """Return (left_wheel, right_wheel, done) without going through calc_steering.

    Uses the lookahead point from next_point() and a direct angular-error
    P-controller so steering is never zero due to cur_speed being 0 at startup.
    """
    if shared_data.local_odom_position is None or shared_data.cur_heading is None:
      return None, None, False

    ox, oy = shared_data.local_odom_position
    heading = math.atan2(math.sin(shared_data.cur_heading), math.cos(shared_data.cur_heading))

    if np.linalg.norm(self.final - np.array([ox, oy])) < self.tolerance:
      self.reached_goal = True

    next_pt = self.next_point([ox, oy])
    if next_pt is None:
      return 0.0, 0.0, self.did_reach_goal()

    dx = next_pt[0] - ox
    dy = next_pt[1] - oy
    desired_ang = math.atan2(dy, dx)
    ang_diff = math.atan2(math.sin(desired_ang - heading), math.cos(desired_ang - heading))

    max_steer = shared_data.AUTON_SPEED * 0.9
    steering_output = clamp(shared_data.STEERING_CONST * ang_diff, -max_steer, max_steer)

    left  = clamp(shared_data.AUTON_SPEED - steering_output, -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
    right = clamp(shared_data.AUTON_SPEED + steering_output, -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
    return left, right, self.did_reach_goal()

  def step(self, shared_data) -> bool:
    """Calls calc_velocity, publishes the result, and returns done."""
    left, right, done = self.calc_velocity(shared_data)
    if left is None:
      return False
    vel = Twist()
    vel.linear.x = (left + right) / 2.0
    vel.angular.z = (right - left) / 1.53
    shared_data.auton_publisher.publish(vel)
    return done

  def check_should_redo_goal(self, cur_pose):
    return not self.reached_goal

  def is_outside_lookahead(self):
    return self.outside_lookahead


  def distance(self, pt1, pt2):
    return np.linalg.norm(np.array(pt1) - np.array(pt2))



  def next_point(self, cur_pt):
    # Strategy:
    # 1) Keep moving forward through waypoints (never jump backward).
    # 2) Pick the first waypoint farther than lookahead distance from current pose.
    # 3) If none is far enough, target the final waypoint.
    #
    # This avoids the old "unreached list" bookkeeping, which could become empty
    # early and leave the controller in a bad state.
    target_idx = self.num_waypoints - 1
    for idx in range(self.look_ahead_point_index, self.num_waypoints):
      if self.distance(self.waypoints[idx], cur_pt) > self.look_ahead_distance:
        target_idx = idx
        break
    self.look_ahead_point_index = target_idx

    # Convert waypoint index -> spline parameter t in [0, 1] for curvature sampling.
    # Sampling curvature with raw index is invalid because spline domain is normalized.
    t = self.look_ahead_point_index / max(self.num_waypoints - 1, 1)

    # Dynamic lookahead: tighter curves -> smaller lookahead, straighter path -> larger.
    # Floor curvature to avoid division-by-zero explosions on straight segments.
    curvature_at_cloest = abs(self.curvature(self.path, t))
    curvature_at_cloest = max(curvature_at_cloest, 1e-6)
    self.look_ahead_distance = max(
      min(self.max_look_ahead, self.max_look_ahead / curvature_at_cloest),
      self.min_look_ahead,
    )

    return self.waypoints[self.look_ahead_point_index]

  # def calc_steering(self, cur_pose):
  #   cur_pt = cur_pose[:2]
  #   cur_ang = cur_pose[2]

  #   next_pt = self.next_point(cur_pt)
  #   if next_pt is None: return float("inf") # mean finished. don't steer

  #   if np.linalg.norm(self.final-cur_pt) < self.tolerance:
  #     self.reached_goal = True

  #   diff = next_pt-cur_pt
  #   d = np.linalg.norm(diff)
  #   if d < 1e-6: # safety check
  #     return float("inf")

  #   # more safety checks
  #   des_ang = np.arctan2(diff[1], diff[0])
  #   ang_diff = np.arctan2(np.sin(des_ang-cur_ang), np.cos(des_ang-cur_ang))
  #   sin_ang = np.sin(ang_diff)
  #   if abs(sin_ang) < 1e-6:
  #     return float("inf")

  #   r = d / (2 * sin_ang)
  #   return r # radius of steering

  @staticmethod
  def generate_path(pts, angs, heading_persistence=1):
    # pts = [s[:2], t[:2]] # start and end point
    # angs = [s[2], t[2]] # start and end angles
    n = np.arange(len(pts))/(len(pts)-1) # discrete

    # start and end derivatives in respect to n
    derivatives = heading_persistence*np.transpose([np.cos(angs), np.sin(angs)])
    boundry_condition = ( (1, derivatives[0]) , (1, derivatives[1]) )

    cs = CubicSpline(n, pts, bc_type=boundry_condition)
    return cs # cubic spline function on [0,1]


  @staticmethod
  def curvature(s, t):
    # v = derivative(s, t, order=1)
    # a = derivative(s, t, order=2)
    v = s(t, 1)
    a = s(t, 2)
    # For 2D, cross product is scalar
    return np.abs(v[0]*a[1] - v[1]*a[0]) / np.linalg.norm(v)**3

  @staticmethod
  def radius(s, t):
    return 1/PurePursuit.curvature(s, t)

  @staticmethod
  def normal_vec(s, n):
    # v = derivative(s, n, order=1)
    v = s(n, 1)
    # Rotate vector 90 degrees
    norm = np.array([-v[1], v[0]])
    return norm / np.linalg.norm(norm)

  @staticmethod
  def get_waypoints(s, dist_btw_waypoints):
    def speed(t):
        # v = derivative(s, t, order=1)
        v = s(t, 1)
        return np.linalg.norm(v)
    length = quad(speed, 0, 1)[0]
    num_segments = length / dist_btw_waypoints
    fineness = 1 / num_segments
    n = np.arange(0, 1 + fineness, fineness)
    return s(n)

  @staticmethod
  def build_waypoint_tree(waypoints):
    return KDTree(waypoints)

def generate_spiral(center, theta_max, sparseness):
  pts = [center]
  thetas = np.arange(0, theta_max, 0.5)
  r = 0
  for theta in thetas:
    pts.append(center+np.array([r*np.cos(theta), r*np.sin(theta)]))
    r += sparseness
  return pts

if __name__ == "__main__":
  """ test client """
  pts = generate_spiral(np.array([0, 0]), 8*np.pi, 0.7)
  pp = PurePursuit(pts, start_ang=0, end_ang=1)
