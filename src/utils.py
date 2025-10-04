import math
from gps_to_ned import GPS_2_NED

def on_new_gps_msg(msg, shared_data):
    # Acquire the mutex to ensure thread-safe access to shared_data
    shared_data.mutex.acquire()

    # If the GPS converter is already initialized
    if shared_data.gps_converter is not None:
        # Convert the received GPS coordinates to local XY coordinates
        shared_data.cur_position = shared_data.gps_converter.gps_to_xy(
            msg.latitude, msg.longitude)
        # Append the current position and heading to a debug file for logging
        with open("debug.txt", 'a') as f:
            f.write(f"{shared_data.cur_position[0]}, {shared_data.cur_position[1]}, {shared_data.cur_heading}\n")
        # Add the current position to the position history list
        shared_data.position_history.append(
            shared_data.cur_position)
    else:
        # If the GPS converter is not initialized, create it using the current GPS coordinates as the origin
        shared_data.gps_converter = GPS_2_NED(
            msg.latitude, msg.longitude)
    # Release the mutex after updating shared_data
    shared_data.mutex.release()
    # Print the current position and heading for debugging
    print(f"Updated position: {shared_data.cur_position}, heading: {shared_data.cur_heading}")

def on_new_heading_msg(msg, shared_data):
    # Acquire the mutex to ensure thread-safe access to shared_data
    shared_data.mutex.acquire()

    # Convert the incoming heading message to the desired format (radians, adjusted)
    heading = -(msg.data - 90) # Use the east as center and flip
    if heading > 180:
        heading -= 360
    shared_data.cur_heading = heading * math.pi / 180
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