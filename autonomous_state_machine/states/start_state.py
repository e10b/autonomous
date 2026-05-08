import rclpy
from std_msgs.msg import Float64MultiArray, String, Bool, Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from autonomous_state_machine.states.state_class import State
from autonomous_state_machine.utils import on_new_gps_msg, on_new_heading_msg, on_receive_new_gps_waypoint, on_new_velocity_msg, found_ar_tag, found_object, on_receive_new_obstacle_avoidance_waypoints #, heartbeat_reset

class StartState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        # Print entering state for debugging
        self.shared_data.logger.info("[StartState] Entering startState...")
        led_msg = Float64MultiArray()
        led_msg.data = [255, 0, 0]
        self.shared_data.led_publisher.publish(led_msg)

        vel_msg = Float64MultiArray()
        vel_msg.data = [0, 0]
        self.shared_data.auton_publisher.publish(vel_msg)

        # Clear debug file for fresh debug output
        self.shared_data.logger.info("[StartState] Clearing debug.txt file...")
        with open('debug.txt', 'w') as f:
            f.write('')

    def step(self):
        # Remain in start state; no state transition logic here
        # print("[StartState] Stepping in StartState (no transition).")
        return self.state_instance.start_state
