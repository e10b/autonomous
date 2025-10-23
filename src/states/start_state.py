import rospy
from std_msgs.msg import Float64MultiArray, String, Bool, Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from state_class import State
from utils import *

class StartState(State):
    def __init__(self, shared_data, state_instance):
        super().__init__()
        self.shared_data = shared_data
        self.state_instance = state_instance

    def on_enter(self):
        # Print entering state for debugging
        print("[StartState] Entering startState...")

        # Clear debug file for fresh debug output
        print("[StartState] Clearing debug.txt file...")
        with open('debug.txt', 'w') as f:
            f.write('')

        # Initialize publishers for autonomous commands, current state, and path
        print("[StartState] Initializing publishers...")
        self.shared_data.auton_publisher = rospy.Publisher(
            self.shared_data.AUTON_TOPIC_NAME, Float64MultiArray, queue_size=1)
        self.shared_data.cur_state_publisher = rospy.Publisher(
            self.shared_data.CUR_STATE_TOPIC_NAME, String, queue_size=1)
        self.shared_data.auton_path_publisher = rospy.Publisher(
            self.shared_data.AUTON_PATH_NAME, Float64MultiArray, queue_size=1)

        # Initialize subscribers for all relevant topics
        print("[StartState] Initializing subscribers...")
        rospy.Subscriber(self.shared_data.GPS_TOPIC_NAME, NavSatFix, lambda msg: on_new_gps_msg(msg, self.shared_data))
        rospy.Subscriber(self.shared_data.COMPASS_TOPIC_NAME, Float64, lambda msg: on_new_heading_msg(msg, self.shared_data))
        rospy.Subscriber(self.shared_data.WAYPOINTS_TOPIC_NAME, Float64MultiArray, lambda msg: on_receive_new_gps_waypoint(msg, self.shared_data))
        rospy.Subscriber(self.shared_data.VELOCITY_TOPIC_NAME, TwistStamped, lambda msg: on_new_velocity_msg(msg, self.shared_data))
        rospy.Subscriber(self.shared_data.ARTAG_TOPIC_NAME, Float64MultiArray, lambda msg: found_ar_tag(msg, self.shared_data))
        rospy.Subscriber(self.shared_data.OBJECT_TOPIC_NAME, Float64MultiArray, lambda msg: found_object(msg, self.shared_data))
        rospy.Subscriber(self.shared_data.OBSTACLE_AVOIDANCE_WAYPOINTS_TOPIC_NAME,
                        Float64MultiArray, lambda msg: on_receive_new_obstacle_avoidance_waypoints(msg, self.shared_data))
        rospy.Subscriber(self.shared_data.HEARTBEAT_TOPIC, Bool, lambda msg: heartbeat_reset(msg, self.shared_data))
        print("[StartState] Initialization complete.")

    def step(self):
        # Remain in start state; no state transition logic here
        print("[StartState] Stepping in StartState (no transition).")
        return self.state_instance.start_state
