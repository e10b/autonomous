import math
from time import time
from autonomous_state_machine.shared_data import SharedData
import autonomous_state_machine.config as config
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from robot_localization.srv import FromLL
from custom_msg_types.msg import AutonInput

def on_new_velocity_msg(msg: TwistStamped, shared_data: SharedData) -> None:
    # Acquire the mutex to ensure thread-safe access to shared_data
    shared_data.mutex.acquire()

    # Extract the x and y components of the velocity from the message
    x_speed = msg.twist.linear.x
    y_speed = msg.twist.linear.y

    # Compute the current speed as the magnitude of the velocity vector
    shared_data.cur_speed = math.sqrt(x_speed**2 + y_speed**2)

    # Print the updated speed for debugging
    # print(f"Updated speed: {shared_data.cur_speed} m/s")

    # Release the mutex after updating shared_data
    shared_data.mutex.release()
    
def on_receive_new_auton_input(auton_input: AutonInput, shared_data: SharedData) -> None:
    """
    Autonomous input is in the following form:
    float64 latitude
    float64 longitude
    float32 speed

    uint8 NONE=0
    uint8 BOTTLE=1
    uint8 HAMMER=2
    uint8 MALLET=3
    uint8 ARUCO_0=4
    uint8 ARUCO_1=5
    uint8 ARUCO_2=6
    uint8 ARUCO_3=7
    uint8 ARUCO_4=8

    uint8 object_id
    """
    shared_data.logger.info(f"Received Autonomous Input: {auton_input}")
    with shared_data.mutex:
        shared_data.logger.info(f"cur_position: {shared_data.cur_position}")
        shared_data.logger.info(f"cur_heading: {shared_data.cur_heading}")

        if shared_data.cur_position is None or \
            shared_data.cur_heading is None:
            shared_data.logger.error("Required shared_data fields are not initialized. Exiting function.")
            return

        shared_data.heartbeat_recovery_active = False

        shared_data.logger.info(f"Auton input object_id: {auton_input.object_id}")
        shared_data.should_look_for_ar = auton_input.object_id > 3
        shared_data.logger.info(f"Should_look_for_ar: {shared_data.should_look_for_ar}")
        shared_data.should_look_for_obj = auton_input.object_id > 0 and not shared_data.should_look_for_ar

        if shared_data.should_look_for_ar:
            # AutonInput AR enums are ARUCO_0..ARUCO_4 = 4..8, while
            # /artag_position publishes marker IDs as 0..4.
            shared_data.target_id = int(auton_input.object_id) - 4
            shared_data.target_obj_id = 0
            shared_data.found_obj_flag = False
        elif shared_data.should_look_for_obj:
            shared_data.target_obj_id = int(auton_input.object_id)
            shared_data.target_id = 0
            shared_data.found_ar_flag = False
        else:
            shared_data.target_id = 0
            shared_data.target_obj_id = 0

        shared_data.counter = 0

    # async but its super fast so its ok
    lat, lon = auton_input.latitude, auton_input.longitude
    req = FromLL.Request()
    req.ll_point.latitude = float(lat)
    req.ll_point.longitude = float(lon)
    req.ll_point.altitude = 0.0

    future = shared_data.llclient.call_async(req)

    def _fromll_done(fut):
        try:
            resp = fut.result()
            if resp is None:
                shared_data.logger.error("FromLL service call failed")
                return
            map_pt = resp.map_point
            goal_pos = (map_pt.x, map_pt.y)
            with shared_data.mutex:
                shared_data.new_waypoints = [goal_pos]
                shared_data.logger.info(f"New waypoints: {shared_data.new_waypoints}")
                shared_data.goal_map_xy = goal_pos
                shared_data.goal_dirty  = True
                shared_data.new_waypoints_flag = True
                shared_data.obstacle_avoidance_running = False
                shared_data.new_obstacle_avoidance_waypoints_flag = False
        except Exception as e:
            shared_data.logger.exception("FromLL call failed: %s", e)

    future.add_done_callback(_fromll_done)

def found_ar_tag(msg: Float64MultiArray, shared_data: SharedData) -> None:
    # msg.data = [cX, cY, size, id]; only the detected id decides whether this is our target
    if len(msg.data) != 4:
        shared_data.logger.warn(f"Ignoring malformed AR tag message (expected 4 values, got {len(msg.data)}): {list(msg.data)}")
        return
    detected_id = int(msg.data[-1])
    with shared_data.mutex:
        shared_data.logger.info(f"Received AR tag message: {msg.data}")

        if detected_id != int(shared_data.target_id):
            shared_data.logger.info(f"AR tag id {detected_id} does not match target {int(shared_data.target_id)}.")
            return

        [shared_data.ar_cX, shared_data.ar_cY,
        shared_data.ar_size, shared_data.ar_id] = msg.data
        shared_data.found_ar_flag = True
        shared_data.latest_ar_found = time()
        shared_data.logger.info(f"Found AR tag: id={shared_data.ar_id}, cX={shared_data.ar_cX}, cY={shared_data.ar_cY}, size={shared_data.ar_size}")

def rotate_and_translate_2d(x: float, y: float, pos_x: float, pos_y: float, heading: float) -> tuple[float, float]:
    """
    Transforms (x, y) coordinates relative to the robot to global coordinates
    using the robot's position and heading.
    heading: in radians, 0 is east, pi/2 is north (unit circle).
    https://en.wikipedia.org/wiki/Transformation_matrix?useskin=vector#Rotation
    """
    offset_heading = heading - math.pi / 2
    x_rot = x * math.cos(offset_heading) - y * math.sin(offset_heading)
    y_rot = x * math.sin(offset_heading) + y * math.cos(offset_heading)
    return (x_rot + pos_x, y_rot + pos_y)


# Converts a flat array to a list of 2-tuples representing waypoints.
# Example: [1, 2, 3, 4, 5, 6] -> [(1, 2), (3, 4), (5, 6)]
def convert_arr_to_waypoint_list(arr: list[float]) -> list[tuple[float, float]]:
    out = []
    it = iter(arr)
    for x in it:
        try:
            y = next(it)
            out.append((x, y))
        except StopIteration:
            raise ValueError('array has odd number of elements')
    return out


def found_object(msg: Float64MultiArray, shared_data: SharedData) -> None:
    # msg.data = [cX, cY, width, height, id]
    if len(msg.data) != 5:
        shared_data.logger.warn(f"Ignoring malformed object message (expected 5 values, got {len(msg.data)}): {list(msg.data)}")
        return
    with shared_data.mutex:
        shared_data.logger.info(f"Seeing object id: {msg.data[-1]}, Looking for: {shared_data.target_obj_id}")
        # Check if the detected object matches the target object ID
        if shared_data.target_obj_id == msg.data[-1]:
            [shared_data.obj_cX, shared_data.obj_cY,
            shared_data.obj_width, shared_data.obj_height, shared_data.obj_id] = msg.data
            shared_data.counter += 1
            shared_data.logger.info(f"Found object we were looking for with id: {msg.data[-1]}")
            # If the object has been detected at least twice, set the found flag
            if shared_data.counter >= 2:
                shared_data.found_obj_flag = True
            shared_data.latest_obj_found = time()

def clamp(num: float, minNum: float, maxNum: float) -> float:
    # Clamp the number between minNum and maxNum
    return max(min(num, maxNum), minNum)

def reset_ar_data(shared_data: SharedData) -> None:
    with shared_data.mutex:
        shared_data.should_look_for_ar = False
        shared_data.found_ar_flag      = False

        shared_data.ar_cX = 0
        shared_data.ar_cY = 0
        shared_data.ar_size = 0
        shared_data.ar_id = 0

def replan_to_goal(shared_data: SharedData, reason: str) -> bool:
    """
    Queue a fresh path from the rover's current position to the current goal.

    This is the single entry point for re-planning toward ``goal_map_xy``. It
    does not build a controller itself; it hands off to the normal pipeline:

      1. Sets ``new_waypoints = [goal_map_xy]`` (map frame) and raises
         ``new_waypoints_flag``. The rover's current position doesn't need to
         be stored here — ``_tick``'s conversion to odom frame always prepends
         the live ``local_odom_position`` as the path's start point.
      2. On the next tick, ``_tick`` converts ``new_waypoints`` to odom
         (``new_waypoints_odom``) and switches to ``NewControllerState``.
      3. ``NewControllerState`` builds the controller selected by
         ``config.CONTROLLER_TYPE`` and clears the flag.

    The caller should return ``state_instance.new_controller_state`` when this
    succeeds. The goal itself (``goal_map_xy`` / ``goal_odom``) is not changed.

    Args:
        shared_data: Shared state. The caller must already hold ``shared_data.mutex``.
        reason: Short description of why we are re-planning, used in the log.

    Returns:
        True if a path was queued. False if the current position or goal is
        unknown; an error is logged and shared state is left unchanged.

    Raises:
        RuntimeError: If called without holding ``shared_data.mutex``.
    """
    if not shared_data.mutex.locked():
        raise RuntimeError("replan_to_goal() must be called while holding shared_data.mutex")

    cur_position = shared_data.cur_position  # readiness check only; not stored (see docstring)
    goal = shared_data.goal_map_xy
    if cur_position is None or goal is None:
        shared_data.logger.error(
            f"[replan_to_goal] cannot re-plan ({reason}): "
            f"cur_position={cur_position}, goal_map_xy={goal}"
        )
        return False

    shared_data.new_waypoints = [goal]
    shared_data.new_waypoints_flag = True
    shared_data.logger.info(f"[replan_to_goal] {reason} — re-planning from {cur_position} to goal {goal}")
    return True

def on_heartbeat(msg, shared_data: SharedData) -> None:
    if not config.HEARTBEAT_ENABLED:
        return
    with shared_data.mutex:
        shared_data.last_heartbeat_time = time()

def on_new_local_odom_msg(msg: Odometry, shared_data: SharedData) -> None:
    shared_data.mutex.acquire()
    try:
        shared_data.last_odom_time = time()
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y

        prev = shared_data._prev_local_odom_pos
        if prev is not None:
            delta = math.hypot(ox - prev[0], oy - prev[1])
            shared_data.recal_distance_accumulator += delta
            if (shared_data.recal_distance_accumulator >= config.RECALIBRATION_DISTANCE
                    and shared_data.new_waypoints):
                shared_data.needs_recalibration = True
                shared_data.recal_distance_accumulator = 0.0

        shared_data.local_odom_position = (ox, oy)
        shared_data._prev_local_odom_pos = (ox, oy)
        if shared_data.last_safe_position is None:
            shared_data.last_safe_position = (ox, oy)
        shared_data.odom_ready = True
        shared_data.cur_position = (ox, oy)
        shared_data.position_history.append(shared_data.cur_position)
        
        # Get odometry orientation data
        orientation_q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y),
                         1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z))
        shared_data.cur_heading = yaw
        # print(f"Updated local odometry position: {shared_data.local_odom_position}, heading: {math.degrees(shared_data.cur_heading)} degrees")
    finally:
        shared_data.mutex.release()

