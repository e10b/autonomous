"""
Module: goal_state

Contains the GoalState class used by the autonomous state machine.
This state represents having reached a goal (e.g. the detected object/AR
marker) and handles heartbeat logic and bookkeeping for last-known positions.
"""

from time import time, sleep
from autonomous_state_machine.states.state_class import State
from autonomous_state_machine.utils import reset_ar_data
from std_msgs.msg import Float64MultiArray, String


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

            # If we're instructed to go immediately to heartbeat, release mutex and return.
            # This avoids flashing green/doing goal processing when heartbeat-driven.
            # if self.shared_data.go_to_heartbeat:
            #     return

            # Notify other components that the goal was reached.
            cur_state_msg = String()
            cur_state_msg.data = "goal_reached"
            self.shared_data.cur_state_publisher.publish(cur_state_msg)
            # Stop the rover
            vel_msg_done = Float64MultiArray()
            vel_msg_done.data = [0, 0]
            self.shared_data.auton_publisher.publish(vel_msg_done)

            # If we are in goal state but the last heartbeat is stale (past timeout),
            # wait a short period and then transition to the heartbeat state.
            # Note: the original code performed a busy-wait; replace with short sleeps
            # to avoid pegging the CPU.
            # if not self.shared_data.go_to_heartbeat and self.shared_data.last_heartbeat - time() > self.shared_data.HEARTBEAT_TIMEOUT:
            #     # lazy solution: wait 15 seconds before switching to heartbeat state
            #     wait_time = time() + 15
            #     while wait_time - time() > 0:
            #         # sleep briefly to avoid busy-wait
            #         sleep(0.1)
            #     self.shared_data.go_to_heartbeat = False
            #     return self.shared_data.heartbeat_state

            # If we have a recent heartbeat (within ~30 seconds), record current position
            # to the history of heartbeat positions. This can be used for recovery/return.
            # NOTE: the time comparison follows the original code; ensure the intended
            # inequality direction in shared_data is correct for your application.
            # if self.shared_data.last_heartbeat_time - time() < 30:
            #     self.shared_data.last_heartbeat_pos.append(self.shared_data.cur_position)

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
        vel_msg_done = Float64MultiArray()
        vel_msg_done.data = [0, 0]
        self.shared_data.auton_publisher.publish(vel_msg_done)

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