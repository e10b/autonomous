#!/usr/bin/env python3

"""http://wiki.ros.org/mavros for topic ref"""

import sys
import functools
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray, Float64, String, Bool
from sensor_msgs.msg import NavSatFix
import math
from collections import deque
import numpy as np
from threading import Lock
from time import time, sleep
from utils import *

from gps_to_ned import GPS_2_NED
from pure_pursuit.pp import PurePursuit
from pure_pursuit.spiral_generation import get_spiral

from states.state_class import State
from states.state_class import StateInstance

from states.start_state import StartState
from states.forward_spiral_state import ForwardSpiralState
from states.goal_state import GoalState
from states.new_controller_state import NewControllerState
from states.retry_state import RetryState
from states.new_obstacle_avoidance_controller_state import NewObstacleAvoidanceControllerState
from states.drive_state import DriveState
from states.obstacle_recovery_state import ObstacleRecoveryState
from states.validating_state import ValidatingState
from states.heartbeat_state import HeartbeatState
from states.go_to_object_state import GoToObjectState
from states.charge_forward_state import ChargeForwardState
from states.spin_state import SpinState
from states.backward_spiral_state import BackwardSpiralState
from states.go_to_ar_state import GoToARState
from states.align_state import AlignState
from states.circle_state import CircleState

class SharedData():
    ################ Constant section ################
    RATE = 30
    TOLERANCE = 2
    HIST_POSE_LEN = 60 # used to be 100 but decreased to 60 for time (either is arbitrary)
    AUTON_SPEED = 1.5
    AR_DRIVE_SPEED = 1.0
    OBJ_DRIVE_SPEED = 0.8
    MAX_AUTON_SPEED = 3
    SPIN_SPEED = 1.0
    STEERING_CONST = 4.5
    DRIVE_TIMEOUT = 30
    SPIRAL_RADIUS = 20
    OBJ_SPIRAL_RADIUS = 11
    SPIRAL_COILS = 4
    OBJ_SPIRAL_COILS = 8
    OBJ_TIMEOUT = 5
    AR_TIMEOUT = 20
    # OBJ_TIMEOUT = 20
    SPIRAL_TIMEOUT = 300
    OBJ_SPIRAL_TIMEOUT = 300
    AR_TOLERANCE = 0.04 # size of ar tag we want to see before we stop
    OBJ_H_TOLERANCE = 0.08 # height of object we want to see before we stop
    OBJ_W_TOLERANCE = 0.08 # width of object we want to see before we stop
    AR_TAGS_DIST_TOLERANCE = 0.01 # for checking if distance between two ar tags are too close
    SPIN_TOLERANCE = math.pi / 10.0 # Tolerance for checking if cur angle is within desired angle
    HEARTBEAT_TIMEOUT = (60)*4 # time in seconds after which we return to previous heartbeat coordinate

    """ Alternate Variables from Brandon-Fix
    AUTON_SPEED = 50
    MAX_AUTON_SPEED = 120
    SPIN_SPEED = 50
    STEERING_CONST = 60
    DRIVE_TIMEOUT = 30
    SPIN_TIMEOUT = 15
    AR_TIMEOUT = 30
    AR_TOLERANCE = 0.062  # size of ar tag we want to see before we stop
    # for checking if distance between two ar tags are too close
    AR_TAGS_DIST_TOLERANCE = 0.01
    BACKUP_TIMEOUT = 2
    AUTON_TOPIC_NAME = "/autonomous_drive_train"
    GPS_TOPIC_NAME = "/mavros/global_position/global"
    VELOCITY_TOPIC_NAME = "/mavros/global_position/raw/gps_vel"
    COMPASS_TOPIC_NAME = "/mavros/global_position/compass_hdg"
    WAYPOINTS_TOPIC_NAME = "/rover_waypoint"
    OBSTACLE_AVOIDANCE_WAYPOINTS_TOPIC_NAME = "/obstacle_avoidance/path"
    ARTAG_TOPIC_NAME = "/artag_position"
    """
    BACKUP_TIMEOUT = 2
    AUTON_TOPIC_NAME     = "/autonomous_drive_train/velocity"
    GPS_TOPIC_NAME       = "/mavros/global_position/global"
    VELOCITY_TOPIC_NAME  = "/mavros/global_position/raw/gps_vel"
    COMPASS_TOPIC_NAME   = "/mavros/global_position/compass_hdg"
    WAYPOINTS_TOPIC_NAME = "/rover_waypoint"
    HEARTBEAT_TOPIC      = "/autonomous_heartbeat"
    ARTAG_TOPIC_NAME     = "/artag_position"
    OBJECT_TOPIC_NAME    = "/object_position"
    OBSTACLE_AVOIDANCE_WAYPOINTS_TOPIC_NAME = "/obstacle_avoidance/path"
    CUR_STATE_TOPIC_NAME = "/set_state"
    AUTON_PATH_NAME      = "/auton_path_display"

    ################ One mutex to rule all the following vars ################
    mutex = Lock()  # this is for the entire data class

    ################ Pub and sub section ################
    auton_publisher = None
    cur_state_publisher = None
    auton_path_publisher = None

    ################ global variables ################
    cur_position = None  # current position in x and y converted from gps intput
    cur_heading = None  # current orientation in radians counter clockwise
    cur_speed = 0     # we'll get this from 'global_position/gp_vel', NOT SAME UNITS AS AUTON SPEED
    # store the backup position to go to if we fail to reach the target
    go_home_position = None
    # [(x, y)] used to check if we are actually near the target
    position_history = deque(maxlen=HIST_POSE_LEN)

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
    last_heartbeat_time = time()
    last_heartbeat_pos  = []
    go_to_heartbeat     = False

    # all should_look_for_ar values to synchronize with new_waypoints
    should_look_for_ar = False
    found_ar_flag = False  # if ar tag has been detected, can go to ar tag state
    do_gate_task = True  # if two tags detected, then do gate task
    do_circle_task = False  # if one tag detected, then do circle task

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



shared_data = SharedData()  # the global state object
state_instance = StateInstance()



def run_machine():
    rate = rospy.Rate(shared_data.RATE)

    state_instance.start_state          = StartState(shared_data, state_instance)
    state_instance.new_controller_state = NewControllerState(shared_data, state_instance)
    state_instance.new_obstacle_avoidance_controller_state = NewObstacleAvoidanceControllerState(shared_data, state_instance)
    state_instance.drive_state = DriveState(shared_data, state_instance)
    state_instance.obstacle_recovery_state = ObstacleRecoveryState(shared_data, state_instance)
    # state_instance.obstacle_avoidance_drive_state           = ObstacleAvoidanceDriveState()
    state_instance.validating_state      = ValidatingState(shared_data, state_instance)
    state_instance.goal_state            = GoalState(shared_data, state_instance)
    state_instance.retry_state           = RetryState(shared_data, state_instance)
    state_instance.spin_state            = SpinState(shared_data, state_instance)
    state_instance.forward_spiral_state  = ForwardSpiralState(shared_data, state_instance)
    state_instance.backward_spiral_state = BackwardSpiralState(shared_data, state_instance)
    state_instance.go_to_ar_state        = GoToARState(shared_data, state_instance)
    state_instance.go_to_obj_state       = GoToObjectState(shared_data, state_instance)
    state_instance.align_state           = AlignState(shared_data, state_instance)
    state_instance.charge_forward_state  = ChargeForwardState(shared_data, state_instance)
    state_instance.circle_state          = CircleState(shared_data, state_instance)
    state_instance.heartbeat_state       = HeartbeatState(shared_data, state_instance)

    cur_state = state_instance.start_state
    cur_state.on_enter()
    last_state = cur_state
    # cur_state = state_instance.goal_state

    while not rospy.is_shutdown():
        rate.sleep()
        # if receive new position from ROS, go to new controller state
        if shared_data.new_waypoints_flag:
            print("user sent more stuff!")
            cur_state = state_instance.new_controller_state
        if last_state != cur_state:
            last_state.on_exit()
            last_state = cur_state

            cur_state.on_enter()
        else:
            last_state = cur_state
            cur_state = cur_state.step()

        # print(time(), shared_data.latest_ar_found)
        if time() - shared_data.latest_ar_found > 2:
            shared_data.found_ar_flag = False

if __name__ == "__main__":
    rospy.init_node("purepursuit")
    run_machine()
