#!/usr/bin/env python3

"""http://wiki.ros.org/mavros for topic ref"""

import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Float64, String
from sensor_msgs.msg import NavSatFix
import math
from collections import deque
import numpy as np
from threading import Lock
from time import time

from gps_to_ned import GPS_2_NED
from pure_pursuit.pp import PurePursuit
from spiral_generation import get_spiral


class SharedData():
    ################ Constant section ################
    RATE = 30
    TOLERANCE = 2.5
    HIST_POSE_LEN = 100
    AUTON_SPEED = 50
    MAX_AUTON_SPEED = 120
    SPIN_SPEED = 45
    STEERING_CONST = 50
    DRIVE_TIMEOUT = 20
    SPIN_TIMEOUT = 10
    AR_TIMEOUT = 5
    AR_TOLERANCE = 0.062 # size of ar tag we want to see before we stop
    AR_TAGS_SIZE_RATIO = 2 # for checking if distance between two ar tags are too close
    CIRCLE_TIMEOUT = 7.5
    AUTON_TOPIC_NAME     = "/autonomous_drive_train"
    GPS_TOPIC_NAME       = "/mavros/global_position/global"
    VELOCITY_TOPIC_NAME  = "/mavros/global_position/gp_vel"
    COMPASS_TOPIC_NAME   = "/mavros/global_position/compass_hdg"
    WAYPOINTS_TOPIC_NAME = "/rover_waypoint"
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

    should_look_for_ar = False  # all should_look_for_ar values to synchronize with new_waypoints  
    found_ar_flag      = False  # if ar tag has been detected, can go to ar tag state
    do_gate_task       = False  # if two tags detected, then do gate task

    ar_cX              = 0      # central x position of detected ar tag
    ar_cY              = 0      # central y position of detected ar tag
    ar_size            = 0      # size of ar tag calculated by area
    ar_id              = 0      # id value of detected ar tag

    ar_cX2             = 0      # central x position of detected ar tag 2
    ar_cY2             = 0      # central y position of detected ar tag 2
    ar_size2           = 0      # size of ar tag 2 calculated by area
    ar_id2             = 0      # id value of detected ar tag 2

    target_id          = 1      # id value of desired ar tag

    controller = None  # pure pursuit controller
    start_time = None  # time stamp of when a state was first entered

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
  #  print(gps_waypoint_list)
    """ waypoint_list is an array of repeating values. i.e. [lat, lon, should_look_for_ar, id, speed, lat, lon...]"""
    shared_data.mutex.acquire()
   # print(shared_data.gps_converter, shared_data.cur_position, shared_data.cur_heading)

    if shared_data.gps_converter is None or\
       shared_data.cur_position  is None or\
       shared_data.cur_heading   is None:
       shared_data.mutex.release()
       return
  
    shared_data.go_home_position = shared_data.cur_position  # remember where to go back to
    shared_data.should_look_for_ar = gps_waypoint_list[-3]

    shared_data.new_waypoints = [shared_data.cur_position]  # put the current position into the waypoints too
    for i in range(0, len(gps_waypoint_list.data)):
        if i % 5 == 0:
            shared_data.new_waypoints.append(shared_data.gps_converter.gps_to_xy(gps_waypoint_list.data[i], gps_waypoint_list.data[i+1]))
    print(shared_data.new_waypoints)

    shared_data.new_waypoints_flag = True

    shared_data.mutex.release()


def found_ar_tag(msg):
    # Use shared_data (create new variables?)
    shared_data.mutex.acquire()
    
    if len(msg.data) > 1:
        shared_data.do_gate_task = True
        [shared_data.ar_cX2, shared_data.ar_cY2,
         shared_data.ar_size2, shared_data.ar_id2] = msg.data[1]

    [shared_data.ar_cX, shared_data.ar_cY,
     shared_data.ar_size, shared_data.ar_id] = msg.data[0]
    if shared_data.ar_id != shared_data.target_id:
        shared_data.mutex.release()
        return

    shared_data.found_ar_flag = True

    shared_data.mutex.release()


def clamp(num, minNum, maxNum):
    return max(min(num, maxNum), minNum)


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
    drive_state             = None
    validating_state        = None
    goal_state              = None
    retry_state             = None
    spin_state              = None
    forward_spiral_state    = None
    backward_spiral_state   = None
    go_to_ar_state          = None
    align_state             = None
    charge_forward_state    = None
    left_align_state        = None
    circle_state            = None

state_instance = StateInstance()

class StartState(State):
    def on_enter(self):
        print("Entering startState...")
        shared_data.auton_publisher = rospy.Publisher(shared_data.AUTON_TOPIC_NAME, Float64MultiArray, queue_size=1)
        shared_data.cur_state_publisher = rospy.Publisher(shared_data.CUR_STATE_TOPIC_NAME, String, queue_size=1)
        rospy.Subscriber(shared_data.GPS_TOPIC_NAME, NavSatFix, on_new_gps_msg)
        rospy.Subscriber(shared_data.COMPASS_TOPIC_NAME, Float64, on_new_heading_msg)
        rospy.Subscriber(shared_data.WAYPOINTS_TOPIC_NAME, Float64MultiArray, on_receive_new_gps_waypoint)
        rospy.Subscriber(shared_data.ARTAG_TOPIC_NAME, Float64MultiArray, found_ar_tag)

    def step(self):
        return state_instance.start_state


class NewControllerState(State):
    def on_enter(self):
        print("Entering newControllerState...")
        shared_data.mutex.acquire()
        # diff btw the last two waypoints
        
        diff = np.array(shared_data.new_waypoints[-1]) - np.array(shared_data.new_waypoints[-2])
        next_orientation = math.atan2(diff[1], diff[0])  # calc the angle of the line connecting the last two waypoints
        shared_data.controller = PurePursuit(shared_data.new_waypoints, 
                                             shared_data.cur_heading, 
                                             next_orientation)
        shared_data.new_waypoints_flag = False  # invalidate the flag so don't enter again
        shared_data.mutex.release()
    def step(self):
        return state_instance.drive_state


class DriveState(State):
    def on_enter(self):
        print("Entering driveState...")
        shared_data.start_time = time()

    def step(self):
        shared_data.mutex.acquire()
        cur_pose = [shared_data.cur_position[0], shared_data.cur_position[1], shared_data.cur_heading]
        steering_radius = shared_data.controller.calc_steering(cur_pose)
        steering_output = shared_data.STEERING_CONST * shared_data.cur_speed / steering_radius
        vel_msg = Float64MultiArray()
        vel_msg.data = [shared_data.AUTON_SPEED - steering_output, 
                        shared_data.AUTON_SPEED + steering_output]
        for i in range(len(vel_msg.data)):
            vel_msg.data[i] = clamp(vel_msg.data[i], -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
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
        shared_data.go_home_position = shared_data.cur_position
        shared_data.mutex.release()

    def step(self):
        # wait for 100 values, then average them
        # check if we are still inside the goal, else set up a new controller
        shared_data.mutex.acquire()

        if len(shared_data.position_history) >= shared_data.HIST_POSE_LEN:
            shared_data.cur_position = list(np.array(shared_data.position_history).mean(axis=0))
            shared_data.controller.check_should_redo_goal(shared_data.cur_position)

            if shared_data.should_look_for_ar:
                shared_data.mutex.release()
                return state_instance.spin_state
            
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
    def step(self) -> State:
        blomk()
        return state_instance.goal_state    


class RetryState(State):
    def on_enter(self):
        print("Entering retryState...")
    def step(self) -> State:
        shared_data.mutex.acquire()
        shared_data.new_waypoints = shared_data.go_home_position
        shared_data.new_waypoints.insert(0, shared_data.cur_position)

        shared_data.mutex.release()
        return state_instance.new_controller_state


class SpinState(State):
    def on_enter(self):
        print("Entering SpinState...")

        shared_data.mutex.acquire()
        shared_data.start_time = time()
        shared_data.mutex.release()
    def step(self) -> State:
        shared_data.mutex.acquire()

        print(f"spinning for {round(time()-shared_data.start_time)} seconds")
        vel_msg = Float64MultiArray()
        vel_msg.data = [-shared_data.SPIN_SPEED, shared_data.SPIN_SPEED]
        shared_data.auton_publisher.publish(vel_msg)

        if time() - shared_data.start_time > shared_data.SPIN_TIMEOUT:  # done with spinning
            shared_data.mutex.release()
            return state_instance.forward_spiral_state
        
        shared_data.mutex.release()
        return state_instance.spin_state


class ForwardSpiralState(State):
    def on_enter(self):
        print("Entering ForwardSpiralState...")
        pts = get_spiral(shared_data.cur_position[0], shared_data.cur_position[1], 
                             radius=10, coils=4, step=2)
        pts = pts[3:]
        end_ang = math.atan((pts[-1][1] - pts[-2][1]) /
                            (pts[-1][0] - pts[-2][0]))
        shared_data.controller = PurePursuit(pts=pts, 
                                             start_ang=shared_data.cur_heading, 
                                             end_ang=end_ang)
    def step(self) -> State:
            
        shared_data.mutex.acquire()
        cur_pose = [shared_data.cur_position[0], shared_data.cur_position[1], shared_data.cur_heading]
        steering_radius = shared_data.controller.calc_steering(cur_pose)
        steering_output = shared_data.STEERING_CONST * shared_data.cur_speed / steering_radius
        vel_msg = Float64MultiArray()
        vel_msg.data = [shared_data.AUTON_SPEED - steering_output,
                        shared_data.AUTON_SPEED + steering_output]
        for i in range(len(vel_msg.data)):
            vel_msg.data[i] = clamp(vel_msg.data[i], -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
        shared_data.auton_publisher.publish(vel_msg)

        # print(
        #     "POSE", list(map(lambda x: round(x, 1), cur_pose)),
        #     "STEERING", round(steering_radius, 3),
        #     "GPS AVG", list(map(lambda x: round(x, 6), np.array(gps_history).mean(axis=0))),
        # )

        if shared_data.found_ar_flag:
            shared_data.mutex.release()
            return state_instance.go_to_ar_state

        if shared_data.controller.did_reach_goal():
            shared_data.mutex.release()

            return state_instance.backward_spiral_state

        shared_data.mutex.release()
        return state_instance.forward_spiral_state


class BackwardSpiralState(State):
    def on_enter(self):
        print("Entering BackwardSpiralState...")
        pts = get_spiral(shared_data.cur_position[0], 
                             shared_data.cur_position[1], 
                             radius=10, coils=-4, step=2)
        pts = pts[3:]
        end_ang = math.atan((pts[-1][1] - pts[-2][1]) /
                            (pts[-1][0] - pts[-2][0]))
        shared_data.controller = PurePursuit(pts=pts, 
                                             start_ang=shared_data.cur_heading, 
                                             end_ang=end_ang)
        
    def step(self) -> State:
        shared_data.mutex.acquire()
        cur_pose = [shared_data.cur_position[0], shared_data.cur_position[1], shared_data.cur_heading]
        steering_radius = shared_data.controller.calc_steering(cur_pose)
        steering_output = shared_data.STEERING_CONST * shared_data.cur_speed / steering_radius
        vel_msg = Float64MultiArray()
        vel_msg.data = [shared_data.AUTON_SPEED - steering_output,
                        shared_data.AUTON_SPEED + steering_output]
        for i in range(len(vel_msg.data)):
            vel_msg.data[i] = clamp(vel_msg.data[i], -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
        shared_data.auton_publisher.publish(vel_msg)

        # print(
        #     "POSE", list(map(lambda x: round(x, 1), cur_pose)),
        #     "STEERING", round(steering_radius, 3),
        #     "GPS AVG", list(map(lambda x: round(x, 6), np.array(gps_history).mean(axis=0))),
        # )

        if shared_data.found_ar_flag:
            shared_data.mutex.release()
            return state_instance.go_to_ar_state

        if shared_data.controller.did_reach_goal():
            shared_data.should_look_for_ar = False
            shared_data.mutex.release()
            return state_instance.retry_state

        shared_data.mutex.release()
        return state_instance.backward_spiral_state


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
        vel_msg.data = [
            shared_data.AUTON_SPEED / 3 + off_center * shared_data.STEERING_CONST * 3,
            shared_data.AUTON_SPEED / 3 - off_center * shared_data.STEERING_CONST * 3,
        ]
        for i in range(len(vel_msg.data)):
            vel_msg.data[i] = clamp(vel_msg.data[i], -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
        shared_data.auton_publisher.publish(vel_msg)

        # if the AR tag is big enough in frame, must be close enough
        if abs(shared_data.ar_size) > shared_data.AR_TOLERANCE:
            vel_msg.data = [0, 0]
            shared_data.auton_publisher.publish(vel_msg)
            if shared_data.do_gate_task:
                shared_data.mutex.release()
                return state_instance.circle_state

            shared_data.mutex.release()
            return state_instance.goal_state

        if time() - shared_data.start_time > shared_data.AR_TIMEOUT:
            shared_data.mutex.release()
            return state_instance.spin_state

        shared_data.mutex.release()
        return state_instance.go_to_ar_state


class AlignState(State):
    def on_enter(self):
        print("Entering AlignState...")

        shared_data.mutex.acquire()
        shared_data.start_time = time()
        shared_data.mutex.release()
    def step(self) -> State:
        shared_data.mutex.acquire()

        vel_msg = Float64MultiArray()

        # Spin to try to detect both ar tags
        if shared_data.ar_size <= 0 or\
           shared_data.ar_size2 <= 0:
            if time() - shared_data.start_time < shared_data.SPIN_TIMEOUT:
                vel_msg.data = [-shared_data.SPIN_SPEED, shared_data.SPIN_SPEED]
                shared_data.auton_publisher.publish(vel_msg)
                shared_data.mutex.release()
                return state_instance.align_state
            else:
                return state_instance.left_align_state

        # Try your best to go to center of both
        if shared_data.ar_cX + shared_data.ar_cX/2 >= 550 or\
           shared_data.ar_cX + shared_data.ar_cX/2 <= 450:
            if time() - shared_data.start_time < shared_data.SPIN_TIMEOUT:
                vel_msg.data = [-shared_data.SPIN_SPEED, shared_data.SPIN_SPEED]
                shared_data.auton_publisher.publish(vel_msg)
                shared_data.mutex.release()
                return state_instance.align_state

        # If the distance between 2 tags is too small compared to size, just circle around left gate
        help = max(shared_data.ar_size, shared_data.ar_size2) / min(shared_data.ar_size, shared_data.ar_size2)
        if help > shared_data.AR_TAGS_SIZE_RATIO:
            return state_instance.left_align_state
        
        if shared_data.ar_size > shared_data.AR_TOLERANCE and\
           shared_data.ar_size2 > shared_data.AR_TOLERANCE:
            return state_instance.charge_forward_state

        return state_instance.align_state


class ChargeForwardState(State):
    def on_enter(self):
        print("Entering ChargeForwardState...")


    def step(self) -> State:
        shared_data.mutex.acquire()
        cur_pose = [shared_data.cur_position[0], shared_data.cur_position[1], shared_data.cur_heading]
        vel_msg = Float64MultiArray()
        off_center = (shared_data.ar_cX + shared_data.ar_cX2)/2 - 0.5
        vel_msg.data = [
            shared_data.AUTON_SPEED / 3 + off_center * shared_data.STEERING_CONST * 3,
            shared_data.AUTON_SPEED / 3 - off_center * shared_data.STEERING_CONST * 3,
        ]
        for i in range(len(vel_msg.data)):
            vel_msg.data[i] = clamp(vel_msg.data[i], -shared_data.MAX_AUTON_SPEED, shared_data.MAX_AUTON_SPEED)
        shared_data.auton_publisher.publish(vel_msg)

        if shared_data.ar_size <= 0 and\
           shared_data.ar_size2 <= 0:
            shared_data.mutex.release()
            return state_instance.goal_state

        shared_data.mutex.release()
        return state_instance.charge_forward_state


class LeftAlignState(State):
    def on_enter(self):
        print("Entering LeftAlignState...:c")
    def step(self) -> State:
        return


class CircleState(State):
    def on_enter(self):
        print("Entering CircleState...")

        shared_data.mutex.acquire()
        shared_data.start_time = time()
        shared_data.mutex.release()
    def step(self) -> State:
        vel_msg = Float64MultiArray()

        if time() - shared_data.start_time < shared_data.SPIN_TIMEOUT/4:
            vel_msg.data = [shared_data.SPIN_SPEED, -shared_data.SPIN_SPEED]
            shared_data.auton_publisher.publish(vel_msg)
            shared_data.mutex.release()
            return state_instance.circle_state

        vel_msg.data = [
            shared_data.MAX_AUTON_SPEED/2 - shared_data.SPIN_SPEED,
            shared_data.MAX_AUTON_SPEED/2 + shared_data.SPIN_SPEED,
        ]
        shared_data.auton_publisher.publish(vel_msg)
        shared_data.mutex.release()

        if time() - shared_data.start_time > shared_data.CIRCLE_TIMEOUT:
            shared_data.mutex.release()
            return state_instance.goal_state
        return state_instance.circle_state


def run_machine():
    rate = rospy.Rate(shared_data.RATE)

    state_instance.start_state           = StartState()
    state_instance.new_controller_state  = NewControllerState()
    state_instance.drive_state           = DriveState()
    state_instance.validating_state      = ValidatingState()
    state_instance.goal_state            = GoalState()
    state_instance.retry_state           = RetryState()
    state_instance.spin_state            = SpinState()
    state_instance.forward_spiral_state  = ForwardSpiralState()
    state_instance.backward_spiral_state = BackwardSpiralState()
    state_instance.go_to_ar_state        = GoToARState()
    state_instance.align_state           = AlignState()
    state_instance.charge_forward_state  = ChargeForwardState()
    state_instance.left_align_state      = LeftAlignState()
    state_instance.circle_state          = CircleState()

    cur_state = state_instance.start_state
    cur_state.on_enter()
    last_state = cur_state
    
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