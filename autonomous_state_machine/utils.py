import math
import time
from autonomous_state_machine.gps_to_ned import GPS_2_NED

def on_new_gps_msg(msg, shared_data):
    # Acquire the mutex to ensure thread-safe access to shared_data
    shared_data.mutex.acquire()

    # If the GPS converter is not initialized, create it using the current GPS coordinates as the origin
    if shared_data.gps_converter is None:
        shared_data.gps_converter = GPS_2_NED(msg.latitude, msg.longitude)
        # Set cur_position using the origin
        shared_data.cur_position = shared_data.gps_converter.gps_to_xy(msg.latitude, msg.longitude)
        shared_data.position_history.append(shared_data.cur_position)
        with open("debug.txt", 'a') as f:
            f.write(f"{shared_data.cur_position[0]}, {shared_data.cur_position[1]}, {shared_data.cur_heading}\n")
    else:
        # Convert the received GPS coordinates to local XY coordinates
        shared_data.cur_position = shared_data.gps_converter.gps_to_xy(msg.latitude, msg.longitude)
        with open("debug.txt", 'a') as f:
            f.write(f"{shared_data.cur_position[0]}, {shared_data.cur_position[1]}, {shared_data.cur_heading}\n")
        shared_data.position_history.append(shared_data.cur_position)
    # Release the mutex after updating shared_data
    shared_data.mutex.release()
    # Print the current position and heading for debugging
    print(f"Updated position: {shared_data.cur_position}, heading: {shared_data.cur_heading}")

def on_new_heading_msg(msg, shared_data):
    # Acquire the mutex to ensure thread-safe access to shared_data
    shared_data.mutex.acquire()

    # Convert compass heading (0=N, CW+) to math heading (0=E, CCW+) and normalize.
    heading_deg = 90.0 - float(msg.data)
    heading_deg = ((heading_deg + 180.0) % 360.0) - 180.0
    shared_data.cur_heading = heading_deg * math.pi / 180.0
    # Print the updated heading for debugging
    print(f"Updated heading: {shared_data.cur_heading} radians")

    # Release the mutex after updating shared_data
    shared_data.mutex.release()

def on_new_velocity_msg(msg, shared_data):
    # Acquire the mutex to ensure thread-safe access to shared_data
    shared_data.mutex.acquire()

    # Extract the x and y components of the velocity from the message
    x_speed = msg.twist.linear.x
    y_speed = msg.twist.linear.y

    # Compute the current speed as the magnitude of the velocity vector
    shared_data.cur_speed = math.sqrt(x_speed**2 + y_speed**2)

    # Print the updated speed for debugging
    print(f"Updated speed: {shared_data.cur_speed} m/s")

    # Release the mutex after updating shared_data
    shared_data.mutex.release()

def on_receive_new_gps_waypoint(gps_waypoint_list, shared_data):
    # Print the received GPS waypoint list for debugging
    print("Received GPS waypoint list:", gps_waypoint_list)

    """
    gps_waypoint_list.data is an array of repeating values:
    [lat, lon, should_look_for_ar, id, speed, lat, lon, ...]
    """
    shared_data.mutex.acquire()
    # Print current state for debugging
    print("gps_converter:", shared_data.gps_converter)
    print("cur_position:", shared_data.cur_position)
    print("cur_heading:", shared_data.cur_heading)

    # Check if required shared_data fields are initialized
    if shared_data.gps_converter is None or \
        shared_data.cur_position is None or \
        shared_data.cur_heading is None:
        print("Required shared_data fields are not initialized. Exiting function.")
        shared_data.mutex.release()
        return

    # Remember where to go back to
    shared_data.go_home_position = shared_data.cur_position
    # Extract and store relevant waypoint data
    shared_data.should_look_for_ar = gps_waypoint_list.data[2]
    shared_data.target_id = gps_waypoint_list.data[3]
    shared_data.should_look_for_obj = 0 if gps_waypoint_list.data[5] == 0 else 1
    shared_data.target_obj_id = gps_waypoint_list.data[5] - 1  # Ensure correct indexing

    size_of_waypts = len(gps_waypoint_list.data)
    # Start new waypoints list with current position
    shared_data.new_waypoints = [shared_data.cur_position]
    # Convert each waypoint in the list to local XY and append
    for i in range(len(gps_waypoint_list.data) // size_of_waypts):
        lat = gps_waypoint_list.data[i * size_of_waypts]
        lon = gps_waypoint_list.data[i * size_of_waypts + 1]
        xy = shared_data.gps_converter.gps_to_xy(lat, lon)
        print(f"Converted waypoint {i}: lat={lat}, lon={lon} -> xy={xy}")
        shared_data.new_waypoints.append(xy)

    print("New waypoints:", shared_data.new_waypoints)

    # Set flags for waypoint update and obstacle avoidance
    shared_data.new_waypoints_flag = True
    shared_data.obstacle_avoidance_running = False
    shared_data.new_obstacle_avoidance_waypoints_flag = False

    shared_data.mutex.release()

def found_ar_tag(msg, shared_data):
    # Acquire the mutex to ensure thread-safe access to shared_data
    shared_data.mutex.acquire()
    print("Received AR tag message:", msg.data)

    # Check if the detected AR tag matches the primary target ID
    if shared_data.target_id == msg.data[-1]:
        [shared_data.ar_cX, shared_data.ar_cY,
        shared_data.ar_size, shared_data.ar_id] = msg.data
        shared_data.found_ar_flag = True
        shared_data.latest_ar_found = time.time()
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

def rotate_and_translate_2d(x, y, pos_x, pos_y, heading):
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
def convert_arr_to_waypoint_list(arr):
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


def on_receive_new_obstacle_avoidance_waypoints(obstacle_avoidence_waypoints, shared_data):
    # Acquire the mutex to ensure thread-safe access to shared_data
    shared_data.mutex.acquire()

    # If obstacle avoidance is not currently running, process the new waypoints
    if not shared_data.obstacle_avoidance_running:
        print("Preparing to avoid obstacle")
        # Convert the flat array to a list of 2D waypoints
        waypoint_list = convert_arr_to_waypoint_list(
            obstacle_avoidence_waypoints.data)
        pos = shared_data.cur_position
        if pos is None:
            print("Current position is None")
        # heading of the robot
        heading = shared_data.cur_heading
        print("Received obstacle avoidance waypoint list:", waypoint_list)
        # Transform waypoints from robot-relative to global coordinates
        if pos is None or heading is None:
            return
        abs_waypoint_list = [rotate_and_translate_2d(
            *pt, *pos, heading) for pt in waypoint_list]
        print("Current position:", pos, "Current heading:", heading)
        print("Transformed (global) obstacle avoidance waypoints:", abs_waypoint_list)
        # Select every third waypoint starting from the fourth (index 3)
        shared_data.obstacle_avoidance_new_waypoints = abs_waypoint_list[3::3]
        print("Filtered obstacle avoidance waypoints:", shared_data.obstacle_avoidance_new_waypoints)

        # Set flag indicating new obstacle avoidance waypoints are available
        shared_data.new_obstacle_avoidance_waypoints_flag = True

    # Release the mutex after updating shared_data
    shared_data.mutex.release()

def heartbeat_reset(msg, shared_data):
    # Acquire the mutex to ensure thread-safe access to shared_data
    shared_data.mutex.acquire()
    # Update the last heartbeat time
    shared_data.last_heartbeat_time = time()
    # If last_heartbeat_pos is empty, append the current position
    if not shared_data.last_heartbeat_pos:
        shared_data.last_heartbeat_pos.append(shared_data.cur_position)
    # Release the mutex after updating shared_data
    shared_data.mutex.release()

def found_object(msg, shared_data):
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

def clamp(num, minNum, maxNum):
    # Clamp the number between minNum and maxNum
    return max(min(num, maxNum), minNum)

def reset_ar_data(shared_data):
    # Acquire the mutex to ensure thread-safe access to shared_data
    shared_data.mutex.acquire()

    # Reset AR tag-related flags and data
    shared_data.should_look_for_ar = False  # Synchronize should_look_for_ar with new_waypoints
    shared_data.found_ar_flag      = False  # If AR tag has been detected, can go to AR tag state
    shared_data.do_gate_task       = False  # If two tags detected, then do gate task
    shared_data.do_circle_task     = False  # If one tag detected, then do circle task

    shared_data.ar_cX = 0      # Central x position of detected AR tag
    shared_data.ar_cY = 0      # Central y position of detected AR tag
    shared_data.ar_size = 0    # Size of AR tag calculated by area
    shared_data.ar_id = 0      # ID value of detected AR tag

    shared_data.ar_cX2 = 0     # Central x position of detected AR tag 2
    shared_data.ar_cY2 = 0     # Central y position of detected AR tag 2
    shared_data.ar_size2 = 0   # Size of AR tag 2 calculated by area
    shared_data.ar_id2 = 0     # ID value of detected AR tag 2

    # Release the mutex after resetting AR data
    shared_data.mutex.release()
