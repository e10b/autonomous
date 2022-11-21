#!/usr/bin/env python3

"""http://wiki.ros.org/mavros for topic ref"""

import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import NavSatFix
import math
from collections import deque
import numpy as np
from threading import Lock

from gps_to_ned import GPS_2_NED
from pp import PurePursuit

class SharedData():
    ################ Constant section ################
    RATE = 30
    TOLERANCE = 2.5
    HIST_POSE_LEN = 100
    AUTON_SPEED = 50
    STEERING_CONST = 50
    AUTON_TOPIC_NAME     = "/autonomous_drive_train"
    GPS_TOPIC_NAME       = "/mavros/global_position/global"
    VELOCITY_TOPIC_NAME  = "/mavros/global_position/gp_vel"
    COMPASS_TOPIC_NAME   = "/mavros/global_position/compass_hdg"
    WAYPOINTS_TOPIC_NAME = "/rover_waypoints"
    ARTAG_TOPIC_NAME     = "/artag_position"
    CUR_STATE_TOPIC_NAME = "/set_state"

    ################ One mutex to rule all the following vars ################
    mutex = Lock()  # this is for the entire data class

    ################ Pub and sub section ################
    auton_publisher     = None
    cur_state_publisher = None

    ################ global variables ################
    cur_position     = None  # current position in x and y converted from gps intput
    cur_heading      = None  # current orientation in radians counter clockwise
    cur_speed        = 0     # we'll get this from 'global_position/gp_vel'
    go_home_position = None  # store the backup position to go to if we fail to reach the target
    position_history = deque(maxlen=HIST_POSE_LEN)  # [(x, y)] used to check if we are actually near the target
    
    gps_converter      = None   # given lat and long spits out x and y
    new_waypoints_flag = False  # used as a synchronization flag
    new_waypoints      = []     # all waypoints we need to hit to get to the target

    controller = None  # pure pursuit controller

shared_data = SharedData()  # the global state object


def on_new_gps_msg(msg):
    shared_data.mutex.acquire()

    if shared_data.gps_converter is not None:
        shared_data.cur_position = shared_data.gps_converter.gps_to_xy(msg.latitude, msg.longitude)  # convert to xy
        shared_data.position_history.append(shared_data.cur_position)  # add to history
    else:
        shared_data.gps_converter = GPS_2_NED(msg.latitude, msg.longitude)  # initialize converter
    
    shared_data.mutex.release()


def on_new_heading_msg(msg):
    shared_data.mutex.acquire()

    # print("HEADING", (msg.data*math.pi/180.0)-math.pi/2)
    if msg.data > 180: msg.data -= 360  #TODO wrap. not the best approach but fine in general
    shared_data.cur_heading = -((msg.data*math.pi/180.0)-math.pi/2)  # heading is just a float64
    
    shared_data.mutex.release()


def on_new_velocity_msg(msg):
    shared_data.mutex.acquire()

    x_speed = msg.twist.linear.x
    y_speed = msg.twist.linear.y
    shared_data.cur_speed = math.sqrt(x_speed**2 + y_speed**2)

    shared_data.mutex.release()


def on_receive_new_gps_waypoint(gps_waypoint_list):
    """ waypoint_list is 2d multi-array nx2. i.e. [[lat, lon]...]"""
    shared_data.mutex.acquire()

    if shared_data.gps_converter is None or\
       shared_data.cur_position  is None or\
       shared_data.cur_heading   is None:
       shared_data.mutex.release()
       return
  
    shared_data.go_home_position = shared_data.cur_position  # remember where to go back to
    
    new_waypoints = [shared_data.cur_position]  # put the current position into the waypoints too
    for coord in gps_waypoint_list:
        new_waypoints.append(shared_data.gps_converter.gps_to_xy(coord[0], coord[1]))
    # print(new_waypoints)

    shared_data.new_waypoints_flag = True

    shared_data.mutex.release()

class State():
    def __init__():
        pass
    def on_enter(self):
        pass
    def on_exit(self):
        print()
        pass
    def step(self):
        pass

class StateInstance():
    start_state = None
    new_controller_state = None
    drive_state = None
    validating_state = None
    goal_state = None

state_instance = StateInstance()

class StartState(State):
    def on_enter(self):
        print("Entering startState...")
        shared_data.auton_publisher = rospy.Publisher(shared_data.AUTON_TOPIC_NAME, Float64MultiArray, queue_size=1)
        shared_data.cur_state_publisher = rospy.Publisher(shared_data.CUR_STATE_TOPIC_NAME, String, queue_size=1)
        rospy.Subscriber(shared_data.GPS_TOPIC_NAME, NavSatFix, on_new_gps_msg)
        rospy.Subscriber(shared_data.COMPASS_TOPIC_NAME, Float64, on_new_heading_msg)
        rospy.Subscriber(shared_data.WAYPOINTS_TOPIC_NAME, Float64MultiArray, on_receive_new_gps_waypoint)
        # rospy.Subscriber(shared_data.ARTAG_TOPIC_NAME, Float64MultiArray, found_ar_tag)

    def step(self):
        return state_instance.goal_state


class NewControllerState(State):
    def on_enter(self):
        print("Entering newControllerState...")
        shared_data.mutex.acquire()
        # diff btw the last two waypoints
        diff = np.array(shared_data.new_waypoints[-1]) - np.array(shared_data.new_waypoints[-2])
        next_orientation = math.atan2(diff[1], diff[0])  # calc the angle of the line connecting the last two waypoints
        shared_data.controller = PurePursuit(shared_data.new_waypoints, shared_data.cur_heading, next_orientation)
        shared_data.new_waypoints_flag = False  # invalidate the flag so don't enter again
        shared_data.mutex.release()
    def step(self):
        return state_instance.drive_state


class DriveState(State):
    def on_enter(self):
        print("Entering driveState...")

    def step(self):
        shared_data.mutex.acquire()
        shared_data.controller.calc_steering(shared_data.cur_position)
        steering_radius = shared_data.controller.calc_steering(shared_data.cur_position, shared_data.cur_heading)
        steering_output = shared_data.STEERING_CONST * shared_data.cur_speed / steering_radius
        vel_msg = Float64MultiArray()
        vel_msg.data = [shared_data.AUTON_SPEED-steering_output, shared_data.AUTON_SPEED+steering_output]
        shared_data.auton_publisher.publish(vel_msg)
        # print(
        #      "CURR POSE", list(map(lambda x: round(x, 3), shared_data.cur_position, shared_data.cur_heading)),
        #      "CURR STEERING", round(steering_radius, 3),
        #      "HIST POSE", list(map(lambda x: round(x, 9), np.array(shared_data.gps_history).mean(axis=0)))
        # )

        # Failed to turn in time
        if shared_data.controller.is_outside_lookahead():
            shared_data.new_waypoints = shared_data.new_waypoints[-1:]
            shared_data.new_waypoints.insert(0, shared_data.cur_position)
            shared_data.mutex.release()
            return state_instance.new_controller_state
        if shared_data.controller.did_reach_goal():
            shared_data.mutex.release()
            return state_instance.validating_state
        shared_data.mutex.release()
        return state_instance.drive_state


class ValidatingState(State):
    def on_enter(self):
        print("Entering validatingState...")
        print("Clearing position_history...")
        shared_data.mutex.acquire()
        shared_data.position_history.clear()
        shared_data.mutex.release()

    def step(self):
        # wait for 100 values, then average them
        # check if we are still inside the goal, else set up a new controller
        shared_data.mutex.acquire()

        if len(shared_data.position_history) >= shared_data.HIST_POSE_LEN:
            shared_data.cur_position = list(np.array(shared_data.position_history).mean(axis=0))
            shared_data.controller.check_should_redo_goal(shared_data.cur_position)
            if not shared_data.controller.did_reach_goal():
                shared_data.new_waypoints = shared_data.new_waypoints[-1:]
                shared_data.new_waypoints.insert(0, shared_data.cur_position)
                shared_data.mutex.release()
                return state_instance.new_controller_state
            else:
                shared_data.mutex.release()
                return state_instance.goal_state

        shared_data.mutex.release()
        return state_instance.validating_state


class GoalState(State):
    def on_enter(self):
        print("Entering goalState...")
    def step(self) -> State():
        return state_instance.goal_state    

def run_machine():
    rate = rospy.rate(shared_data.RATE)

    state_instance.start_state = StartState()
    state_instance.new_controller_state = NewControllerState()
    state_instance.drive_state = DriveState()
    state_instance.validating_state = ValidatingState()
    state_instance.goal_state = GoalState()

    cur_state = state_instance.start_state
    last_state = State()
    
    while not rospy.is_shutdown():
        rate.sleep()
        # if receive new position from ROS, go to new controller state
        if shared_data.new_waypoints_flag:
            cur_state = state_instance.new_controller_state
        if type(last_state).__name__ != type(cur_state).__name__:
            last_state.on_exit()
            last_state = cur_state

            cur_state.on_enter()
        else:
            last_state = cur_state
            cur_state = cur_state.step()

if __name__ == "__main__":
    rospy.init_node("purepursuit")
    run_machine()
