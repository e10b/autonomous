<<<<<<< HEAD
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
=======
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

from states.start_state import StartState
from states.obstacle_recovery_state import ObstacleRecoveryState
from states.new_controller_state import NewControllerState
from states.new_obstacle_avoidance_controller_state import NewObstacleAvoidanceControllerState
from states.drive_state import DriveState
from states.validating_state import ValidatingState

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


class State():
    def __init__(self):
        pass

    def on_enter(self):
        pass

    def on_exit(self):
        pass

    def step(self):
        pass


class StateInstance():
    start_state             = None
    new_controller_state    = None
    new_obstacle_avoidance_controller_state = None
    drive_state             = None
    obstacle_recovery_state = None
    validating_state        = None
    goal_state              = None
    retry_state             = None
    spin_state              = None
    forward_spiral_state    = None
    backward_spiral_state   = None
    go_to_ar_state          = None
    go_to_obj_state         = None
    align_state             = None
    charge_forward_state    = None
    circle_state            = None
    heartbeat_state         = None

state_instance = StateInstance()

class GoalState(State):
    def on_enter(self):
        print("Entering goalState...")

        shared_data.mutex.acquire()

        if shared_data.found_ar_flag:
            print(shared_data.ar_cX, shared_data.found_ar_flag)
        elif shared_data.found_obj_flag:
            print(shared_data.obj_cX, shared_data.found_obj_flag)

        # don't flash green when returning via heartbeat, just sit in this loop til new pts
        if shared_data.go_to_heartbeat:
            shared_data.mutex.release()
            return

        #print("We got to the bottle cuz it was big enough on screen")
        shared_data.cur_state_publisher.publish("goal_reached")

        # we in goalstate but heartbeat has passed
        if not shared_data.go_to_heartbeat and shared_data.last_heartbeat - time() > shared_data.HEARTBEAT_TIMEOUT:
            # lazy solution is wait 15 seconds
            wait_time = time() + 15
            while(wait_time - time() > 0):
                pass # no op, this can definitely be improved this is just busy waiting
            shared_data.go_to_heartbeat = False
            shared_data.mutex.release()
            return shared_data.heartbeat_state

        # add this goal state position if we have recieved heartbeat
        if shared_data.last_heartbeat_time - time() < 30:
                shared_data.last_heartbeat_pos.append(shared_data.cur_position)        

        shared_data.mutex.release()

    def step(self) -> State:
        reset_ar_data()
        return state_instance.goal_state

class RetryState(State):
    def on_enter(self):
        print("Entering retryState...")

    def step(self) -> State:
        shared_data.mutex.acquire()
        shared_data.new_waypoints = [shared_data.go_home_position]
        shared_data.new_waypoints.insert(0, shared_data.cur_position)
        shared_data.should_look_for_ar = False

        shared_data.mutex.release()
        return state_instance.new_controller_state


class SpinState(State):
    def on_enter(self):
        print("Entering SpinState...")

        shared_data.mutex.acquire()
        shared_data.start_time = time()
        shared_data.start_heading = shared_data.cur_heading
        shared_data.mutex.release()

    def step(self) -> State:
        shared_data.mutex.acquire()

        print(f"spinning for {round(time()-shared_data.start_time)} seconds")
        vel_msg = Float64MultiArray()
        vel_msg.data = [-shared_data.SPIN_SPEED, shared_data.SPIN_SPEED]
        shared_data.auton_publisher.publish(vel_msg)

        if not shared_data.go_to_heartbeat and shared_data.last_heartbeat - time() > shared_data.HEARTBEAT_TIMEOUT:
            shared_data.mutex.release()
            return shared_data.heartbeat_state

        if shared_data.found_ar_flag and shared_data.should_look_for_ar:
            shared_data.mutex.release()
            return state_instance.go_to_ar_state

        if shared_data.found_obj_flag and shared_data.should_look_for_obj:
            shared_data.mutex.release()
            return state_instance.go_to_obj_state

        if time() - shared_data.start_time > 1 and \
           abs(shared_data.cur_heading - shared_data.start_heading) < shared_data.SPIN_TOLERANCE:  # done with spinning
            shared_data.mutex.release()
            return state_instance.forward_spiral_state

        shared_data.mutex.release()
        return state_instance.spin_state


class ForwardSpiralState(State):
    def on_enter(self):
        print("Entering ForwardSpiralState...")
        # sleep(3)
        shared_data.mutex.acquire()
        shared_data.start_time = time()
        if shared_data.should_look_for_ar:
            pts = get_spiral(shared_data.cur_position[0], shared_data.cur_position[1], 
                                 radius=shared_data.SPIRAL_RADIUS, coils=shared_data.SPIRAL_COILS, step=2, direction=1)
        # pts = [(0, 0), (0, 3), (-0.5, 3), (-1, -4)]
        if shared_data.should_look_for_obj:
            pts = get_spiral(shared_data.cur_position[0], shared_data.cur_position[1], 
                                 radius=shared_data.OBJ_SPIRAL_RADIUS, coils=shared_data.OBJ_SPIRAL_COILS, step=2, direction=1)
        end_ang = math.atan((pts[-1][1] - pts[-2][1]) /
                            (pts[-1][0] - pts[-2][0]))
        shared_data.controller = PurePursuit(pts=pts, 
                                             start_ang=shared_data.cur_heading, 
                                             end_ang=end_ang)
        shared_data.mutex.release()

    def step(self) -> State:
        shared_data.mutex.acquire()
        cur_pose = [shared_data.cur_position[0], shared_data.cur_position[1], shared_data.cur_heading]
        steering_radius = shared_data.controller.calc_steering(cur_pose)
        steering_output = shared_data.STEERING_CONST * shared_data.AUTON_SPEED / steering_radius / 2
        vel_msg = Float64MultiArray()
        vel_msg.data = [shared_data.AUTON_SPEED - steering_output,
                        shared_data.AUTON_SPEED + steering_output]
        for i in range(len(vel_msg.data)):
            vel_msg.data[i] = clamp(vel_msg.data[i], -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
        shared_data.auton_publisher.publish(vel_msg)

        #print(f"LEFTWHEEL{vel_msg.data[0]} \
                #RIGHTWHEEL{vel_msg.data[1]}")
        if not shared_data.go_to_heartbeat and shared_data.last_heartbeat - time() > shared_data.HEARTBEAT_TIMEOUT:
            shared_data.mutex.release()
            return shared_data.heartbeat_state

        if shared_data.found_ar_flag and shared_data.should_look_for_ar:
            shared_data.mutex.release()
            return state_instance.go_to_ar_state

        if shared_data.found_obj_flag and shared_data.should_look_for_obj:
            shared_data.mutex.release()
            return state_instance.go_to_obj_state

        if shared_data.controller.did_reach_goal():
            shared_data.mutex.release()
            return state_instance.backward_spiral_state

        if (shared_data.should_look_for_ar and (time() - shared_data.start_time > shared_data.SPIRAL_TIMEOUT)):  # done with spiraling
        # if shared_data.controller.did_reach_goal() or \
        #    time() - shared_data.start_time > shared_data.AR_TIMEOUT:
            shared_data.mutex.release()
            return state_instance.backward_spiral_state

        if (shared_data.should_look_for_obj and (time() - shared_data.start_time > shared_data.OBJ_SPIRAL_TIMEOUT)):  # done with spiraling
        # if shared_data.controller.did_reach_goal() or \
        #    time() - shared_data.start_time > shared_data.AR_TIMEOUT:
            shared_data.mutex.release()
            return state_instance.backward_spiral_state


        shared_data.mutex.release()
        return state_instance.forward_spiral_state


class BackwardSpiralState(State):
    def on_enter(self):
        print("Entering BackwardSpiralState...")
        shared_data.mutex.acquire()
        shared_data.start_time = time()
        if shared_data.should_look_for_ar:
            pts = get_spiral(shared_data.cur_position[0], shared_data.cur_position[1], 
                                 radius=shared_data.SPIRAL_RADIUS, coils=shared_data.SPIRAL_COILS, step=2, direction=-1)
        if shared_data.should_look_for_obj:
            pts = get_spiral(shared_data.cur_position[0], shared_data.cur_position[1], 
                                 radius=shared_data.OBJ_SPIRAL_RADIUS, coils=shared_data.OBJ_SPIRAL_COILS, step=2, direction=-1)
        pts = pts[3:]
        end_ang = math.atan((pts[-1][1] - pts[-2][1]) /
                            (pts[-1][0] - pts[-2][0]))
        shared_data.controller = PurePursuit(pts=pts, 
                                             start_ang=shared_data.cur_heading, 
                                             end_ang=end_ang)
        shared_data.mutex.release()

    def step(self) -> State:
        shared_data.mutex.acquire()
        cur_pose = [shared_data.cur_position[0], shared_data.cur_position[1], shared_data.cur_heading]
        steering_radius = shared_data.controller.calc_steering(cur_pose)
        steering_output = shared_data.STEERING_CONST * shared_data.AUTON_SPEED / steering_radius / 2
        vel_msg = Float64MultiArray()
        vel_msg.data = [shared_data.AUTON_SPEED - steering_output,
                        shared_data.AUTON_SPEED + steering_output]
        for i in range(len(vel_msg.data)):
            vel_msg.data[i] = clamp(vel_msg.data[i], -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
        shared_data.auton_publisher.publish(vel_msg)

        # print(
        #      "CURR POS", round(shared_data.cur_position[0], 3), round(shared_data.cur_position[1], 3),
        #      "FIN POS", round(shared_data.new_waypoints[-1][0], 3), round(shared_data.new_waypoints[-1][1], 3),
        #      "LEFT WHEEL", round(vel_msg.data[0], 3),
        #      "RIGHT WHEEL", round(vel_msg.data[1], 3),
        # )
        if not shared_data.go_to_heartbeat and shared_data.last_heartbeat - time() > shared_data.HEARTBEAT_TIMEOUT:
            shared_data.mutex.release()
            return shared_data.heartbeat_state

        if shared_data.found_ar_flag and shared_data.should_look_for_ar:
            shared_data.mutex.release()
            return state_instance.go_to_ar_state
        
        if shared_data.found_obj_flag and shared_data.should_look_for_obj:
            shared_data.mutex.release()
            return state_instance.go_to_obj_state

        if shared_data.controller.did_reach_goal():
            shared_data.mutex.release()
            return state_instance.backward_spiral_state

        if (shared_data.should_look_for_ar and (time() - shared_data.start_time > shared_data.SPIRAL_TIMEOUT)):  # done with spiraling
        # if shared_data.controller.did_reach_goal() or \
        #    time() - shared_data.start_time > shared_data.AR_TIMEOUT:
            shared_data.mutex.release()
            return state_instance.retry_state

        if (shared_data.should_look_for_obj and (time() - shared_data.start_time > shared_data.OBJ_SPIRAL_TIMEOUT)):  # done with spiraling
        # if shared_data.controller.did_reach_goal() or \
        #    time() - shared_data.start_time > shared_data.AR_TIMEOUT:
            shared_data.mutex.release()
            return state_instance.retry_state
        

        shared_data.mutex.release()
        return state_instance.backward_spiral_state

class HeartbeatState(State):
    def on_enter(self):
        print("Entering Heartbeat State")
        shared_data.mutex.acquire()
        shared_data.start_time = time()
        shared_data.new_waypoints = [shared_data.last_heartbeat_pos[-1]]
        shared_data.new_waypoints.insert(0, shared_data.cur_position)
        shared_data.go_to_heartbeat = True
        shared_data.mutex.release()
        return state_instance.new_controller_state

class GoToARState(State):
    def on_enter(self):
        print("Entering goToARState...")

        shared_data.mutex.acquire()
        shared_data.start_time = time()
        shared_data.mutex.release()

    def step(self) -> State:
        shared_data.mutex.acquire()

        vel_msg = Float64MultiArray()

        off_center = shared_data.ar_cX - 0.5
        print(off_center, shared_data.ar_size)
        vel_msg.data = [
            shared_data.AR_DRIVE_SPEED + off_center * 1.5,
            shared_data.AR_DRIVE_SPEED - off_center * 1.5,
        ]

        for i in range(len(vel_msg.data)):
            vel_msg.data[i] = clamp(
                vel_msg.data[i], -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
        shared_data.auton_publisher.publish(vel_msg)

        # print(f"LEFT WHEEL: {vel_msg.data[0]} \
        #        RIGHT WHEEL: {vel_msg.data[1]}")

        # if the AR tag is big enough in frame, must be close enough

        # if abs(shared_data.ar_size) > shared_data.AR_TOLERANCE and False:
        if not shared_data.go_to_heartbeat and shared_data.last_heartbeat - time() > shared_data.HEARTBEAT_TIMEOUT:
            shared_data.mutex.release()
            return shared_data.heartbeat_state

        if abs(shared_data.ar_size) > shared_data.AR_TOLERANCE:
            vel_msg.data = [0, 0]
            shared_data.auton_publisher.publish(vel_msg)
            shared_data.found_ar_flag = False
            if shared_data.do_gate_task:
                shared_data.do_gate_task = False
                shared_data.mutex.release()
                return state_instance.circle_state

            shared_data.mutex.release()
            return state_instance.goal_state

        if time() - shared_data.start_time > shared_data.AR_TIMEOUT:
            shared_data.mutex.release()
            return state_instance.spin_state

        shared_data.mutex.release()
        return state_instance.go_to_ar_state

class GoToObjectState(State):
    def on_enter(self):
        shared_data.counter = 0
        print("Entering goToObjectState...")
        shared_data.mutex.acquire()
        shared_data.start_time = time()
        shared_data.mutex.release()

    def step(self) -> State:
        shared_data.mutex.acquire()

        off_center = shared_data.obj_cX - 0.5
        print(off_center, shared_data.obj_width, shared_data.obj_height)
        vel_msg = Float64MultiArray()
        vel_msg.data = [
            shared_data.OBJ_DRIVE_SPEED + off_center * 1.5,
            shared_data.OBJ_DRIVE_SPEED - off_center * 1.5,
        ]

        #steering_adjustment = off_center * shared_data.STEERING_CONST
        #vel_msg = Float64MultiArray()
        #vel_msg.data = [
        #    shared_data.AUTON_SPEED - steering_adjustment,
        #    shared_data.AUTON_SPEED + steering_adjustment
        #]

        # Publish velocity commands
        shared_data.auton_publisher.publish(vel_msg)  # check if we need different publisher here, ADD

        if not shared_data.go_to_heartbeat and shared_data.last_heartbeat - time() > shared_data.HEARTBEAT_TIMEOUT:
            shared_data.mutex.release()
            return shared_data.heartbeat_state

        # Decide if the object is close enough to stop
        if shared_data.obj_width > shared_data.OBJ_W_TOLERANCE or shared_data.obj_height > shared_data.OBJ_H_TOLERANCE :
            vel_msg.data = [0, 0]  # Stop
            shared_data.auton_publisher.publish(vel_msg)
            shared_data.found_obj_flag = False  # Reset flag
            shared_data.mutex.release()
            return state_instance.goal_state  # Transition to goal state
        
        if time() - shared_data.start_time > shared_data.OBJ_TIMEOUT:
            shared_data.mutex.release()
            return state_instance.spin_state

        shared_data.mutex.release()
        return state_instance.go_to_obj_state

class AlignState(State):
    def on_enter(self):
        print("Entering AlignState...")

        shared_data.mutex.acquire()
        shared_data.start_time = time()
        shared_data.start_heading = shared_data.cur_heading
        shared_data.mutex.release()

    def step(self) -> State:
        shared_data.mutex.acquire()

        vel_msg = Float64MultiArray()

        # Back up for 2 seconds to get a better view of the gate
        if time() - shared_data.start_time < shared_data.BACKUP_TIMEOUT:
            vel_msg.data = [-shared_data.AUTON_SPEED, -shared_data.AUTON_SPEED]
            shared_data.auton_publisher.publish(vel_msg)
            shared_data.mutex.release()
            return state_instance.align_state

        # Spin util both tags are in frame
        if shared_data.ar_size <= 0 or\
           shared_data.ar_size2 <= 0:
            """if time() - shared_data.start_time < 1 + shared_data.BACKUP_TIMEOUT and\
               abs(shared_data.cur_heading - shared_data.start_heading) < shared_data.SPIN_TOLERANCE:
                vel_msg.data = [-shared_data.SPIN_SPEED, shared_data.SPIN_SPEED]"""
            if time() - shared_data.start_time < shared_data.SPIN_TIMEOUT + shared_data.BACKUP_TIMEOUT:
                vel_msg.data = [-shared_data.SPIN_SPEED,
                                shared_data.SPIN_SPEED]
                shared_data.auton_publisher.publish(vel_msg)
                if shared_data.ar_size > 0:
                    print("Saw AR Tag Id: 0.0")
                if shared_data.ar_size2 > 0:
                    print("Saw AR Tag ID: 1.0")
                shared_data.mutex.release()
                return state_instance.align_state
            else:
                shared_data.mutex.release()
                return state_instance.circle_state

        # Spin until both tags are centered
        if (shared_data.ar_x_pose + shared_data.ar_x_pose2)/2 >= 0.6 or\
           (shared_data.ar_x_pose + shared_data.ar_x_pose2)/2 <= 0.4:
            if time() - shared_data.start_time < shared_data.SPIN_TIMEOUT + shared_data.BACKUP_TIMEOUT:
                vel_msg.data = [-shared_data.SPIN_SPEED,
                                shared_data.SPIN_SPEED]
                shared_data.auton_publisher.publish(vel_msg)
                shared_data.mutex.release()
                return state_instance.align_state

        # If the distance between 2 tags is too small or only see 1 tag, go back to that tag
        distbtwntags = abs(shared_data.ar_x_pose - shared_data.ar_x_pose2)
        if distbtwntags < shared_data.AR_TAGS_DIST_TOLERANCE or\
           shared_data.ar_size <= 0.01 or\
           shared_data.ar_size2 <= 0.01:
            shared_data.do_circle_task = True
            print("Should do circle")
            shared_data.mutex.release()
            return state_instance.spin_state

        shared_data.mutex.release()
        return state_instance.charge_forward_state


class ChargeForwardState(State):
    def on_enter(self):
        print("Entering ChargeForwardState...")

    def step(self) -> State:
        shared_data.mutex.acquire()
        vel_msg = Float64MultiArray()
        off_center = (shared_data.ar_x_pose + shared_data.ar_x_pose2)/2 - 0.5
        vel_msg.data = [
            shared_data.AUTON_SPEED / 3 + off_center * shared_data.STEERING_CONST * 3,
            shared_data.AUTON_SPEED / 3 - off_center * shared_data.STEERING_CONST * 3,
        ]
        for i in range(len(vel_msg.data)):
            vel_msg.data[i] = clamp(
                vel_msg.data[i], -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
        shared_data.auton_publisher.publish(vel_msg)

        #print(f"LEFT WHEEL: {vel_msg.data[0]} \
        #        RIGHT WHEEL: {vel_msg.data[1]}")

        if shared_data.ar_size > 0.01 or\
           shared_data.ar_size2 > 0.01:
            shared_data.start_time = time()

        print(f"Running for {round(time()-shared_data.start_time, 3)} seconds")

        if time() - shared_data.start_time > 2:
            vel_msg.data = [0, 0]
            shared_data.auton_publisher.publish(vel_msg)
            shared_data.found_ar_flag = False
            shared_data.mutex.release()
            return state_instance.goal_state

        shared_data.mutex.release()
        return state_instance.charge_forward_state


class CircleState(State):
    def on_enter(self):
        print("Entering CircleState...")

        shared_data.mutex.acquire()
        shared_data.start_time = time()
        shared_data.start_heading = shared_data.cur_heading
        shared_data.mutex.release()

        self.done_perp = False
    def step(self) -> State:
        shared_data.mutex.acquire()
        vel_msg = Float64MultiArray()

        # Spin 90 degrees CW to be perpendicular to the AR Tag pole
        if not self.done_perp and abs(shared_data.cur_heading - (shared_data.start_heading - math.pi/2)) > shared_data.SPIN_TOLERANCE:
            print(f"Spinning 90 deg for {round(time()-shared_data.start_time, 3)} seconds { abs(shared_data.cur_heading - (shared_data.start_heading - math.pi/2))}")
            """if time() - shared_data.start_time < shared_data.SPIN_TIMEOUT/4:
            print(
                f"Spinning 90 deg for {round(time()-shared_data.start_time, 3)} seconds")"""
            vel_msg.data = [shared_data.SPIN_SPEED, -shared_data.SPIN_SPEED]
            shared_data.auton_publisher.publish(vel_msg)
            shared_data.mutex.release()
            return state_instance.circle_state
        else:
            self.done_perp = True
        print(f"Spinning 270 deg for {round(time()-shared_data.start_time, 3)} seconds")

        # vel_msg.data = [
        #     shared_data.MAX_AUTON_SPEED/2 - shared_data.SPIN_SPEED/2,
        #     shared_data.MAX_AUTON_SPEED/2 + shared_data.SPIN_SPEED/2,
        # ]
        vel_msg.data = [
            0.5,
            2,
        ]
        shared_data.auton_publisher.publish(vel_msg)

        end_angle = shared_data.start_heading + 2/3 * math.pi if shared_data.start_heading < 0 \
                    else shared_data.start_heading - 2/3 * math.pi
        # Circle 270 degrees to get around the AR Tag pole
        if abs(shared_data.cur_heading - end_angle) < shared_data.SPIN_TOLERANCE:
            shared_data.found_ar_flag = False
            shared_data.mutex.release()
            return state_instance.goal_state

        shared_data.mutex.release()
        return state_instance.circle_state


def run_machine():
    rate = rospy.Rate(shared_data.RATE)

    state_instance.start_state          = StartState(shared_data, state_instance)
    state_instance.new_controller_state = NewControllerState(shared_data, state_instance)
    state_instance.new_obstacle_avoidance_controller_state = NewObstacleAvoidanceControllerState(shared_data, state_instance)
    state_instance.drive_state = DriveState(shared_data, state_instance)
    state_instance.obstacle_recovery_state = ObstacleRecoveryState(shared_data, state_instance)
    # state_instance.obstacle_avoidance_drive_state           = ObstacleAvoidanceDriveState()
    state_instance.validating_state      = ValidatingState(shared_data, state_instance)
    state_instance.goal_state            = GoalState()
    state_instance.retry_state           = RetryState()
    state_instance.spin_state            = SpinState()
    state_instance.forward_spiral_state  = ForwardSpiralState()
    state_instance.backward_spiral_state = BackwardSpiralState()
    state_instance.go_to_ar_state        = GoToARState()
    state_instance.go_to_obj_state       = GoToObjectState()
    state_instance.align_state           = AlignState()
    state_instance.charge_forward_state  = ChargeForwardState()
    state_instance.circle_state          = CircleState()
    state_instance.heartbeat_state       = HeartbeatState()

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
>>>>>>> f6b400a074c7f75701e5a8e09071decf7a107ac6
