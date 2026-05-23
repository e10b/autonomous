"""
Module: goal_state

Contains the GoalState class used by the autonomous state machine.
This state represents having reached a goal (e.g. the detected object/AR
marker) and handles heartbeat logic and bookkeeping for last-known positions.
"""

from time import time
from autonomous_state_machine.states.state_class import State
from autonomous_state_machine.utils import reset_ar_data
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist


class GoalState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance
        self.counter = 0

    def on_enter(self):
        self.log("Entering goalState...")

        # Protect access to shared data
        with self.shared_data.mutex:
            # Debug: print which detector triggered (AR marker vs object)
            if self.shared_data.found_ar_flag:
                self.log(f"{self.shared_data.ar_cX} {self.shared_data.found_ar_flag}")
            elif self.shared_data.found_obj_flag:
                self.log(f"{self.shared_data.obj_cX} {self.shared_data.found_obj_flag}")

            # Notify other components that the goal was reached.
            cur_state_msg = String()
            cur_state_msg.data = "goal_reached"
            self.shared_data.cur_state_publisher.publish(cur_state_msg)
            # Stop the rover
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0
            self.shared_data.auton_publisher.publish(stop)

    def step(self) -> State:
        """
        Called repeatedly while in GoalState. Clean up any AR-related temporary data
        and remain in the goal state (or trigger a transition by returning a different state).

        Returns:
            The next state to transition to (here, remain in goal_state).
        """
        # Clear AR detection buffers to avoid stale detections carrying over.
        reset_ar_data(self.shared_data)

        # Stop the rover
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.shared_data.auton_publisher.publish(stop)

        led_msg = Float64MultiArray()

        # Increment
        if self.counter == 50:
            led_msg.data = [0, 255, 0]
            self.shared_data.led_publisher.publish(led_msg)
        elif self.counter == 100:
           led_msg.data = [0, 0, 0]
           self.shared_data.led_publisher.publish(led_msg)
        elif self.counter > 100:
            self.counter = 0

        self.counter += 1

        return self.state_instance.goal_state