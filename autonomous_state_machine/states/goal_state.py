"""
Module: goal_state

Contains the GoalState class used by the autonomous state machine.
This state represents having reached a goal (e.g. the detected object/AR
marker) and handles heartbeat logic and bookkeeping for last-known positions.
"""

from time import time, sleep
from autonomous_state_machine.states.state_class import State
from autonomous_state_machine.utils import reset_ar_data


class GoalState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        print("Entering goalState...")

        # Protect access to shared data
        self.shared_data.mutex.acquire()

        # Debug: print which detector triggered (AR marker vs object)
        if self.shared_data.found_ar_flag:
            print(self.shared_data.ar_cX, self.shared_data.found_ar_flag)
        elif self.shared_data.found_obj_flag:
            print(self.shared_data.obj_cX, self.shared_data.found_obj_flag)

        # If we're instructed to go immediately to heartbeat, release mutex and return.
        # This avoids flashing green/doing goal processing when heartbeat-driven.
        if self.shared_data.go_to_heartbeat:
            self.shared_data.mutex.release()
            return

        # Notify other components that the goal was reached.
        self.shared_data.cur_state_publisher.publish("goal_reached")

        # If we are in goal state but the last heartbeat is stale (past timeout),
        # wait a short period and then transition to the heartbeat state.
        # Note: the original code performed a busy-wait; replace with short sleeps
        # to avoid pegging the CPU.
        if not self.shared_data.go_to_heartbeat and self.shared_data.last_heartbeat - time() > self.shared_data.HEARTBEAT_TIMEOUT:
            # lazy solution: wait 15 seconds before switching to heartbeat state
            wait_time = time() + 15
            while wait_time - time() > 0:
                # sleep briefly to avoid busy-wait
                sleep(0.1)
            self.shared_data.go_to_heartbeat = False
            self.shared_data.mutex.release()
            return self.shared_data.heartbeat_state

        # If we have a recent heartbeat (within ~30 seconds), record current position
        # to the history of heartbeat positions. This can be used for recovery/return.
        # NOTE: the time comparison follows the original code; ensure the intended
        # inequality direction in shared_data is correct for your application.
        if self.shared_data.last_heartbeat_time - time() < 30:
            self.shared_data.last_heartbeat_pos.append(self.shared_data.cur_position)

        # Release mutex before leaving on_enter
        self.shared_data.mutex.release()

    def step(self, state_instance) -> State:
        """
        Called repeatedly while in GoalState. Clean up any AR-related temporary data
        and remain in the goal state (or trigger a transition by returning a different state).

        Args:
            state_instance: container of states used for transitions (e.g., state_instance.goal_state)

        Returns:
            The next state to transition to (here, remain in goal_state).
        """
        # Clear AR detection buffers to avoid stale detections carrying over.
        reset_ar_data()
        return state_instance.goal_state