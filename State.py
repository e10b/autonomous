#!/usr/bin/env python3
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.misc import derivative
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

from pp import PurePursuit
from collections import deque

# global variable for gps signal
# global variable for vel_msg
class State():
    cur_pose = None
    nxt_pose = None
    fin_pose = None
    controller = None
    pose_history = deque(maxlen=100)

    def on_enter(self):
        pass
    def on_exit(self):
        print()
        pass
    def step(self):
        pass

class mainState(State):
    def on_enter(self):
        print("Entering mainState...")
        finalx = float(input("Input the final x coordinate: "))
        finaly = float(input("Input the final y coordinate: "))
        State.fin_pose = [finalx, finaly, 0]
        
        # intialize node pure pursuit
        # initialize pub subbers
        pass
    def step(self):
        return sendState

class sendState(State):
    def on_enter(self) -> State():
        print("Entering sendState...")
        # get some sort of xy position
        pass
    def step(self) -> State():
        # if ROSInterruptException
        #     return mainState
        # if controller and cur pose does not exist
        #     return sendState
        # if reached goal
        #     return validatingState()
        # else if target goes out of camera frame
        #     return lookoutState()

        x = float(input("Input our current x coordinate: "))
        y = float(input("Input our current y coordinate: "))
        State.cur_pose = [x, y, 0]
        State.pose_history.append(self.cur_pose)
        State.controller = PurePursuit(self.cur_pose, self.nxt_pose)

        cur_pt = np.array(self.cur_pose[:2])
        fin_pt = np.array(self.nxt_pose[:2])
        if np.linalg.norm(fin_pt-cur_pt) < 1.5:
            return validatingState
        if self.controller.is_outside_lookahead():
            return lookoutState
        return driveState

class validatingState(State):
    def on_enter(self) -> State():
        print("Entering validatingState...")
        print("Clearing cur_pose...")
        State.pose_history.clear()
        # stop motors
        pass
    def step(self) -> State():
        # wait for 100 values
        # if reached 100 values
        #     avg them
        #     check if should redo goal
        #         return sendState
        #     return goalState
        State.cur_pose = [x, y, 0]
        State.pose_history.append(self.cur_pose)

        if len(self.pose_history) >= 5:
            State.cur_pose = list(np.array(self.pose_history).mean(axis=0))
            cur_pt = np.array(self.cur_pose[:2])
            fin_pt = np.array(self.nxt_pose[:2])
            if np.linalg.norm(fin_pt-cur_pt) >= 3:
                return sendState  # redo the last goal point
            return goalState
        return validatingState

class goalState(State):
    def on_enter(self):
        print("Entering goalState...")
    def step(self) -> State():
        # if received a new gps signal, use new xy positions
        #     return mainState
        if str(input("Did we get a new signal (y/n): ")) == 'y':
            return mainState
        return goalState

class lookoutState(State):
    def on_enter(self):
        print("Entering lookoutState...")
        State.controller = PurePursuit(self.cur_pose, self.nxt_pose)
    def step(self):
        return mainState

class driveState(State):
    def on_enter(self):
        print("Entering driveState...")
        # calculate steering radius and output
        # set vel_msg.data
        pass
    def step(self):
        self.controller.calc_steering(self.cur_pose)
        return sendState

def Machine():
    cur_state = mainState()
    last_state = State()
    
    while True:
        if type(last_state).__name__ != type(cur_state).__name__:
            last_state.on_exit()
            last_state = cur_state

            cur_state.on_enter()
        else:
            cur_state = cur_state.step()()
        # send vel_msg

if __name__ == "__main__":
    machine = Machine()