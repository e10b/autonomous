import math
from time import time
from autonomous_state_machine.gps_to_ned import GPS_2_NED
from autonomous_state_machine.shared_data import SharedData
import rclpy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from robot_localization.srv import FromLL

def on_new_gps_msg(msg: NavSatFix, shared_data: SharedData) -> None:
    with shared_data.mutex:
        if shared_data.gps_converter is None:
            shared_data.gps_converter = GPS_2_NED(msg.latitude, msg.longitude)
        shared_data.cur_position = shared_data.gps_converter.gps_to_xy(msg.latitude, msg.longitude)
        shared_data.position_history.append(shared_data.cur_position)
        shared_data.gps_ready = True
        with open("debug.txt", 'a') as f:
            f.write(f"{shared_data.cur_position[0]}, {shared_data.cur_position[1]}, {shared_data.cur_heading}\n")

def on_new_heading_msg(msg: Imu, shared_data: SharedData) -> None:
    with shared_data.mutex:
        q = msg.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        shared_data.cur_heading_raw = yaw
        shared_data.heading_ready = True

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

def on_receive_new_gps_waypoint(gps_waypoint_list: Float64MultiArray, shared_data: SharedData) -> None:
    """
    gps_waypoint_list.data is an array of repeating values:
    [lat, lon, should_look_for_ar, id, speed, should_look_for_obj, lat, lon, lat, lon, ...]
    """
    shared_data.logger.info(f"Received GPS waypoint list: {gps_waypoint_list}")
    shared_data.mutex.acquire()
    try:
        shared_data.logger.info(f"gps_converter: {shared_data.gps_converter}")
        shared_data.logger.info(f"cur_position: {shared_data.cur_position}")
        shared_data.logger.info(f"cur_heading: {shared_data.cur_heading}")

        if shared_data.gps_converter is None or \
            shared_data.cur_position is None or \
            shared_data.cur_heading is None:
            shared_data.logger.error("Required shared_data fields are not initialized. Exiting function.")
            return

        data = gps_waypoint_list.data
        shared_data.go_home_position = shared_data.cur_position
        shared_data.should_look_for_ar = data[2]
        shared_data.target_id = data[3]
        # data[4] = speed (unused here), data[5] = should_look_for_obj flag
        shared_data.should_look_for_obj = 0 if len(data) <= 5 or data[5] == 0 else 1
        shared_data.target_obj_id = (data[5] - 1) if len(data) > 5 else 0

        # Waypoints start at index 6, each waypoint is [lat, lon]
        HEADER_SIZE = 6
        shared_data.new_waypoints = [shared_data.cur_position]
        i = HEADER_SIZE
        while i + 1 < len(data):
            lat, lon = data[i], data[i + 1]
            xy = shared_data.gps_converter.gps_to_xy(lat, lon)
            shared_data.logger.info(f"Converted waypoint: lat={lat}, lon={lon} -> xy={xy}")
            shared_data.new_waypoints.append(xy)
            i += 2

        # Fall back to header lat/lon if no waypoints in the tail
        if len(shared_data.new_waypoints) == 1:
            xy = shared_data.gps_converter.gps_to_xy(data[0], data[1])
            shared_data.new_waypoints.append(xy)

        shared_data.logger.info(f"New waypoints: {shared_data.new_waypoints}")
        shared_data.goal_map_xy = tuple(shared_data.new_waypoints[-1])
        shared_data.goal_dirty  = True
        shared_data.new_waypoints_flag = True
        shared_data.obstacle_avoidance_running = False
        shared_data.new_obstacle_avoidance_waypoints_flag = False
    finally:
        shared_data.mutex.release()

def found_ar_tag(msg: Float64MultiArray, shared_data: SharedData) -> None:
    # Acquire the mutex to ensure thread-safe access to shared_data
    shared_data.mutex.acquire()
    shared_data.logger.info(f"Received AR tag message: {msg.data}")

    # Check if the detected AR tag matches the primary target ID
    if shared_data.target_id == msg.data[-1]:
        [shared_data.ar_cX, shared_data.ar_cY,
        shared_data.ar_size, shared_data.ar_id] = msg.data
        shared_data.found_ar_flag = True
        shared_data.latest_ar_found = time()
        print(f"Found primary AR tag: id={shared_data.ar_id}, cX={shared_data.ar_cX}, cY={shared_data.ar_cY}, size={shared_data.ar_size}")

    # Check if the detected AR tag matches the secondary target ID
    if hasattr(shared_data, 'target_id2') and shared_data.target_id2 == msg.data[3]:
        [shared_data.ar_cX2, shared_data.ar_cY2,
        shared_data.ar_size2, shared_data.ar_id2] = msg.data
        print(f"Found secondary AR tag: id2={shared_data.ar_id2}, cX2={shared_data.ar_cX2}, cY2={shared_data.ar_cY2}, size2={shared_data.ar_size2}")

    # If the found AR tag does not match the primary target, release mutex and exit
    if shared_data.ar_id != shared_data.target_id:
        print("AR tag ID does not match target. Exiting function.")
        shared_data.mutex.release()
        return

    # Set flag indicating AR tag was found
    shared_data.found_ar_flag = True
    print("AR tag found flag set to True.")

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
    print("Seeing object id:", msg.data[-1], "Looking for:", shared_data.target_obj_id)
    # Check if the detected object matches the target object ID
    if shared_data.target_obj_id == msg.data[-1]:
        [shared_data.obj_cX, shared_data.obj_cY,
        shared_data.obj_width, shared_data.obj_height, shared_data.obj_id] = msg.data
        shared_data.counter += 1
        print("Found object we were looking for with id:", msg.data[-1])
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

def on_new_local_odom_msg(msg: Odometry, shared_data: SharedData) -> None:
    shared_data.mutex.acquire()
    try:
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        shared_data.local_odom_position = (ox, oy)
        shared_data._prev_local_odom_pos = (ox, oy)
        shared_data.odom_ready = True
        
        # Get odometry orientation data
        orientation_q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y),
                         1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z))
        shared_data.cur_heading = yaw
        # print(f"Updated local odometry position: {shared_data.local_odom_position}, heading: {math.degrees(shared_data.cur_heading)} degrees")
    finally:
        shared_data.mutex.release()

