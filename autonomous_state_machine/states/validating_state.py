import numpy as np
from time import time
from autonomous_state_machine.states.state_class import State
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import autonomous_state_machine.config as config

class ValidatingState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        with self.shared_data.mutex:
            in_recovery = config.HEARTBEAT_ENABLED and self.shared_data.heartbeat_recovery_active

        if in_recovery:
            self.log("ENTER (recovery) — at last safe position, waiting for heartbeat to restore", logtype="warn")
        else:
            self.log(f"ENTER — stopped, collecting {self.shared_data.HIST_POSE_LEN} position samples to verify goal")

        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.shared_data.auton_publisher.publish(stop)

        with self.shared_data.mutex:
            self.shared_data.position_history.clear()

    def step(self):
        if config.HEARTBEAT_ENABLED:
            with self.shared_data.mutex:
                in_recovery = self.shared_data.heartbeat_recovery_active
                hb_time     = self.shared_data.last_heartbeat_time
            if in_recovery:
                if hb_time > 0 and time() - hb_time < config.HEARTBEAT_TIMEOUT:
                    self.log("heartbeat restored — clearing recovery flag, → StartState")
                    with self.shared_data.mutex:
                        self.shared_data.heartbeat_recovery_active = False
                    return self.state_instance.start_state
                return self.state_instance.validating_state

        # wait for 100 values, then average them
        # check if we are still inside the goal, else set up a new controller
        with self.shared_data.mutex:
            if len(self.shared_data.position_history) >= self.shared_data.HIST_POSE_LEN:
                self.shared_data.cur_position = list(
                    np.array(self.shared_data.position_history).mean(axis=0))
                self.shared_data.controller.check_should_redo_goal(
                    self.shared_data.cur_position)

                if self.shared_data.should_look_for_ar:
                    self.log("position sampled — AR search active, → SpinState")
                    return self.state_instance.spin_state

                if self.shared_data.should_look_for_obj:
                    self.log("position sampled — object search active, → SpinState")
                    return self.state_instance.spin_state

                if not self.shared_data.controller.did_reach_goal():
                    self.log("goal not confirmed at averaged position — re-planning → NewControllerState", logtype="warn")
                    self.shared_data.new_waypoints = self.shared_data.new_waypoints[-1:]
                    self.shared_data.new_waypoints.insert(0, self.shared_data.cur_position)
                    return self.state_instance.new_controller_state
                else:
                    self.log("goal confirmed — → GoalState")
                    return self.state_instance.goal_state

            return self.state_instance.validating_state
