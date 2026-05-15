import math
from collections import deque
from threading import Lock

class SharedData():
    ################ Constant section ################
    

    HIST_POSE_LEN = 5 # used to be 100 but decreased to 60 for time (either is arbitrary)
    AUTON_SPEED = 1.5
    MAX_AUTON_SPEED = 3
    SPIN_SPEED = 1.0
    STEERING_CONST = 4.5
    SPIRAL_RADIUS = 20
    OBJ_SPIRAL_RADIUS = 11
    SPIRAL_COILS = 4
    OBJ_SPIRAL_COILS = 8
    
    SEGMENT_LENGTH   = 3.0
    LOCAL_ODOM_TOPIC  = "/odometry/local"
    GLOBAL_ODOM_TOPIC = "/odometry/global"

    AR_TIMEOUT = 20
    SPIRAL_TIMEOUT = 300
    OBJ_SPIRAL_TIMEOUT = 300
    AR_TOLERANCE = 0.067 # size of ar tag we want to see before we stop
    OBJ_H_TOLERANCE = 0.08 # height of object we want to see before we stop
    OBJ_W_TOLERANCE = 0.08 # width of object we want to see before we stop
    AR_TAGS_DIST_TOLERANCE = 0.01 # for checking if distance between two ar tags are too close
    SPIN_TOLERANCE = math.pi / 10.0 # Tolerance for checking if cur angle is within desired angle
    # HEARTBEAT_TIMEOUT = (60)*4 # time in seconds after which we return to previous heartbeat coordinate
    BACKUP_TIMEOUT = 2
    AUTON_TOPIC_NAME     = "/autonomous_drive_train/velocity"
    GPS_TOPIC_NAME       = "/mavros/global_position/global"
    VELOCITY_TOPIC_NAME  = "/mavros/global_position/raw/gps_vel"
    COMPASS_TOPIC_NAME   = "/mavros/global_position/compass_hdg"
    WAYPOINTS_TOPIC_NAME = "/rover_waypoint"
    # HEARTBEAT_TOPIC      = "/autonomous_heartbeat"
    ARTAG_TOPIC_NAME     = "/artag_position"
    OBJECT_TOPIC_NAME    = "/object_position"
    OBSTACLE_AVOIDANCE_WAYPOINTS_TOPIC_NAME = "/obstacle_avoidance/path"
    CUR_STATE_TOPIC_NAME = "/set_state"
    AUTON_PATH_NAME      = "/auton_path_display"
    LED_TOPIC_NAME       = "/led"

    ################ One mutex to rule all the following vars ################
    mutex = Lock()  # this is for the entire data class

    ################ global variables ################
    cur_position = None  # current position in x and y converted from gps intput
    cur_heading = None  # current orientation in radians counter clockwise
    cur_speed = 0     # we'll get this from 'global_position/gp_vel', NOT SAME UNITS AS AUTON SPEED
    gps_ready = False
    heading_ready = False
    # store the backup position to go to if we fail to reach the target
    go_home_position = None
    # [(x, y)] used to check if we are actually near the target
    position_history = deque(maxlen=HIST_POSE_LEN)

    odom_ready            = False
    global_odom_ready     = False
    local_odom_position   = None       # (x, y) in odom frame
    global_odom_position  = None       # (x, y) in map frame
    _prev_local_odom_pos  = None       # for delta distance accumulation
    odom_bias             = (0.0, 0.0) # translation: local_odom frame → GPS_2_NED frame
    odom_bias_set         = False
    gps_ned_to_map_offset = None       # (dx, dy) offset from map frame to GPS_2_NED frame, set once at startup
    segment_distance      = 0.0        # meters traveled since last recalibration

    gps_converter = None   # given lat and long spits out x and y
    gps_time = None   # timer to register time between new gps receipts
    new_waypoints_flag = False  # used as a synchronization flag
    new_waypoints      = []     # all waypoints we need to hit to get to the target
    counter             = 0     # see how many times it sees the object

    should_look_for_obj = False  # self explanatory
    should_look_for_ar  = False  # all should_look_for_ar values to synchronize with new_waypoints
    found_ar_flag       = False  # if ar tag has been detected, can go to ar tag state
    found_obj_flag      = False  # if object has been detect, can go to object state
    do_gate_task        = False  # if two tags detected, then do gate task
    do_circle_task      = False  # if one tag detected, then do circle task
    latest_ar_found     = 0      # time stamp for latest ar tag found
    latest_obj_found    = 0      # time stamp for latest object found

    obj_cX             = 0      # central x position of detected object
    obj_cY             = 0      # central y position of detected object
    obj_width           = 0      # width of object
    obj_height          = 0      # height of object
    obj_id             = 0.0      # id value of object to distinguish mallet and bottle

    new_obstacle_avoidance_waypoints_flag = False  # used as a synchronization flag
    # all waypoints we need to hit to get to the target
    obstacle_avoidance_new_waypoints = []
    # obstacle avoidance has run and can avoid another obstacle
    obstacle_avoidance_running = False

    # heartbeat time and pos
    # last_heartbeat_time = time()
    # last_heartbeat_pos  = []
    # go_to_heartbeat     = False

    # all should_look_for_ar values to synchronize with new_waypoints
    # FIXME: these values are declared twice. What's the correct init value?
    # should_look_for_ar = False
    # found_ar_flag = False  # if ar tag has been detected, can go to ar tag state
    # do_gate_task = True  # if two tags detected, then do gate task
    # do_circle_task = False  # if one tag detected, then do circle task

    ar_cX = 0      # central x position of detected ar tag
    ar_cY = 0      # central y position of detected ar tag
    ar_size = 0      # size of ar tag calculated by area
    ar_id = 0      # id value of detected ar tag

    ar_cX2 = 0      # central x position of detected ar tag 2
    ar_cY2 = 0      # central y position of detected ar tag 2
    ar_size2 = 0      # size of ar tag 2 calculated by area
    ar_id2 = 0      # id value of detected ar tag 2

    target_id          = 0      # id value of desired ar tag
    target_id2         = 0.0    # id value of desired ar tag 2 for gate task
    #in brandon's version it was 1
    target_obj_id      = 1.0      # id value of desired object 1 = mallet 0 = bottle

    controller    = None  # pure pursuit controller
    start_time    = None  # time stamp of when a state was first entered
    start_heading = None  # heading stamp to mark when we end spinning
    obstacle_avoidance_controller = None

    global_map           = None   # GlobalMapManager instance
    last_oa_trigger_time = 0.0   # timestamp of last obstacle avoidance trigger
    logger               = None