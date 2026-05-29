import math
from time import time
from autonomous_state_machine.gps_to_ned import GPS_2_NED
from autonomous_state_machine.shared_data import SharedData
import autonomous_state_machine.config as config
import rclpy
from sensor_msgs.msg import NavSatFix, Imu
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
    
def on_receive_new_gps_waypoint_llclient(gps_waypoint_list: Float64MultiArray, shared_data: SharedData) -> None:
    shared_data.logger.info(f"Received GPS waypoint list for LL client: {gps_waypoint_list}")

    data = gps_waypoint_list.data
    if len(data) < 2:
        shared_data.logger.error("Received GPS waypoint list is too short.")
        return

    lat, lon = data[0], data[1]

    req = FromLL.Request()
    req.ll_point.latitude = float(lat)
    req.ll_point.longitude = float(lon)
    req.ll_point.altitude = 0.0

    future = shared_data.llclient.call_async(req)

    def _fromll_done(fut):
        try:
            resp = fut.result()
            if resp is None:
                shared_data.logger.error("FromLL returned None")
                return
            map_pt = resp.map_point
            with shared_data.mutex:
                xy = (map_pt.x, map_pt.y)
                shared_data.new_waypoints = [shared_data.cur_position, xy]
                shared_data.goal_map_xy = xy
                shared_data.goal_dirty = True
                shared_data.new_waypoints_flag = True
                shared_data.logger.info(f"FromLL: lat={lat}, lon={lon} -> xy={xy}")
        except Exception as e:
            shared_data.logger.exception("FromLL call failed: %s", e)

    future.add_done_callback(_fromll_done)

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

        shared_data.go_home_position = shared_data.cur_position
        shared_data.heartbeat_recovery_active = False
        start_pos = shared_data.cur_position  # capture before async gap

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
                shared_data.new_waypoints = [start_pos, goal_pos]
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
    # Acquire the mutex to ensure thread-safe access to shared_data
    shared_data.mutex.acquire()
    shared_data.logger.info(f"Received AR tag message: {msg.data}")

    # Check if the detected AR tag matches the primary target ID
    detected_id = int(msg.data[-1])
    if int(shared_data.target_id) == detected_id:
        [shared_data.ar_cX, shared_data.ar_cY,
        shared_data.ar_size, shared_data.ar_id] = msg.data
        shared_data.found_ar_flag = True
        shared_data.latest_ar_found = time()
        shared_data.logger.info(f"Found primary AR tag: id={shared_data.ar_id}, cX={shared_data.ar_cX}, cY={shared_data.ar_cY}, size={shared_data.ar_size}")

    # Check if the detected AR tag matches the secondary target ID
    if hasattr(shared_data, 'target_id2') and int(shared_data.target_id2) == int(msg.data[3]):
        [shared_data.ar_cX2, shared_data.ar_cY2,
        shared_data.ar_size2, shared_data.ar_id2] = msg.data
        shared_data.logger.info(f"Found secondary AR tag: id2={shared_data.ar_id2}, cX2={shared_data.ar_cX2}, cY2={shared_data.ar_cY2}, size2={shared_data.ar_size2}")

    # If the found AR tag does not match the primary target, release mutex and exit
    if int(shared_data.ar_id) != int(shared_data.target_id):
        shared_data.logger.info("AR tag ID does not match target. Exiting function.")
        shared_data.mutex.release()
        return

    # Set flag indicating AR tag was found
    shared_data.found_ar_flag = True
    shared_data.logger.info("AR tag found flag set to True.")

    # Release the mutex after updating shared_data
    shared_data.mutex.release()

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
    print("Converting array to waypoint list:", arr)
    out = []
    it = iter(arr)
    for x in it:
        try:
            y = next(it)
            out.append((x, y))
            print(f"Added waypoint: ({x}, {y})")
        except StopIteration:
            print("Error: array has odd number of elements")
            raise ValueError('array has odd number of elements')
    print("Final waypoint list:", out)
    return out


def on_receive_new_obstacle_avoidance_waypoints(obstacle_avoidence_waypoints: Float64MultiArray, shared_data: SharedData) -> None:
    # Avoidance is now triggered directly from the occupancy grid callback in the
    # state machine node (_on_occupancy_grid), which fires regardless of whether
    # obstacle_detection's local Theta* finds a route.  This callback is kept as a
    # no-op so the subscription doesn't error out but doesn't double-trigger.
    pass

def found_object(msg: Float64MultiArray, shared_data: SharedData) -> None:
    # Acquire the mutex to ensure thread-safe access to shared_data
    shared_data.mutex.acquire()
    shared_data.logger.info("Seeing object id:", msg.data[-1], "Looking for:", shared_data.target_obj_id)
    # Check if the detected object matches the target object ID
    if shared_data.target_obj_id == msg.data[-1]:
        [shared_data.obj_cX, shared_data.obj_cY,
        shared_data.obj_width, shared_data.obj_height, shared_data.obj_id] = msg.data
        shared_data.counter += 1
        shared_data.logger.info("Found object we were looking for with id:", msg.data[-1])
        # If the object has been detected at least twice, set the found flag
        if shared_data.counter >= 2:
            shared_data.found_obj_flag = True
        shared_data.latest_obj_found = time()
    # Release the mutex after updating shared_data
    shared_data.mutex.release()

def clamp(num: float, minNum: float, maxNum: float) -> float:
    # Clamp the number between minNum and maxNum
    return max(min(num, maxNum), minNum)

def reset_ar_data(shared_data: SharedData) -> None:
    with shared_data.mutex:
        shared_data.should_look_for_ar = False
        shared_data.found_ar_flag      = False
        shared_data.do_gate_task       = False
        shared_data.do_circle_task     = False

        shared_data.ar_cX = 0
        shared_data.ar_cY = 0
        shared_data.ar_size = 0
        shared_data.ar_id = 0

        shared_data.ar_cX2 = 0
        shared_data.ar_cY2 = 0
        shared_data.ar_size2 = 0
        shared_data.ar_id2 = 0

def on_heartbeat(msg, shared_data: SharedData) -> None:
    if not config.HEARTBEAT_ENABLED:
        return
    with shared_data.mutex:
        shared_data.last_heartbeat_time = time()

def on_new_local_odom_msg(msg: Odometry, shared_data: SharedData) -> None:
    shared_data.mutex.acquire()
    try:
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
        shared_data.gps_ready = True
        
        # Get odometry orientation data
        orientation_q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y),
                         1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z))
        shared_data.cur_heading = yaw
        # print(f"Updated local odometry position: {shared_data.local_odom_position}, heading: {math.degrees(shared_data.cur_heading)} degrees")
    finally:
        shared_data.mutex.release()

