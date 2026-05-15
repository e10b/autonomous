#!/usr/bin/env python3

# import sys
# print("Python version", sys.version)
# print("Python executable", sys.executable)

"""http://wiki.ros.org/mavros for topic ref"""

from autonomous_state_machine.utils import on_new_gps_msg, on_new_heading_msg, on_receive_new_gps_waypoint, on_new_velocity_msg, found_ar_tag, found_object, on_receive_new_obstacle_avoidance_waypoints, on_new_local_odom_msg, on_new_global_odom_msg#, heartbeat_reset
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float64, String, Bool
from sensor_msgs.msg import NavSatFix
from time import time
from autonomous_state_machine.global_map import GlobalMapManager

# from autonomous_state_machine.gps_to_ned import GPS_2_NED
# from autonomous_state_machine.pp import PurePursuit
# from autonomous_state_machine.spiral_generation import get_spiral

from autonomous_state_machine.states.state_class import State
from autonomous_state_machine.states.state_class import StateInstance

from autonomous_state_machine.states.start_state import StartState
from autonomous_state_machine.states.forward_spiral_state import ForwardSpiralState
from autonomous_state_machine.states.goal_state import GoalState
from autonomous_state_machine.states.new_controller_state import NewControllerState
from autonomous_state_machine.states.retry_state import RetryState
from autonomous_state_machine.states.new_obstacle_avoidance_controller_state import NewObstacleAvoidanceControllerState
from autonomous_state_machine.states.drive_state import DriveState
from autonomous_state_machine.states.obstacle_recovery_state import ObstacleRecoveryState
from autonomous_state_machine.states.validating_state import ValidatingState
# from autonomous_state_machine.states.heartbeat_state import HeartbeatState
from autonomous_state_machine.states.go_to_object_state import GoToObjectState
from autonomous_state_machine.states.spin_state import SpinState
from autonomous_state_machine.states.backward_spiral_state import BackwardSpiralState
from autonomous_state_machine.states.go_to_ar_state import GoToARState
from autonomous_state_machine.states.circle_state import CircleState
from autonomous_state_machine.shared_data import SharedData

RATE = 30

class AutonomousStateMachine(Node):
    def __init__(self):
        super().__init__("autonomous_state_machine")
        self.get_logger().info("Starting autonomous state machine node...")

        self.shared_data = SharedData()  # the global state object
        self.state_instance = StateInstance()

        self.shared_data.node = self
        self.shared_data.logger = self.get_logger()
        self.shared_data.global_map = GlobalMapManager(logger=self.get_logger())
        self.build_states()
        self.setup_subscribers()
        self.setup_publishers()

        self.cur_state = self.state_instance.start_state
        self.last_state = self.cur_state
        self.cur_state.on_enter()

        self._map_print_counter = 0
        self.timer = self.create_timer(1.0 / RATE, self._tick)


    def _on_occupancy_grid(self, msg: Float64MultiArray):
        pos = self.shared_data.cur_position
        yaw = self.shared_data.cur_heading
        if pos is None or yaw is None:
            return
        if len(msg.layout.dim) < 2:
            return
        grid_x = msg.layout.dim[0].size
        grid_z = msg.layout.dim[1].size
        if len(msg.data) != grid_x * grid_z:
            return
        local_grid = np.array(msg.data, dtype=np.uint8).reshape(grid_x, grid_z)
        self.shared_data.global_map.update_map(local_grid, pos[0], pos[1], yaw)

        # Trigger avoidance directly from the occupancy grid so we don't depend on
        # obstacle_detection's local Theta* succeeding.  The occupancy grid is always
        # published; the path topic is only published when the local planner finds a
        # route, so a wide / close obstacle silently drops the trigger.
        _OA_CORRIDOR_HALF = 8   # cells — matches obstacle_detection (ceil(0.75 / 0.10))
        _OA_COOLDOWN      = 3.0 # seconds — slightly longer than reverse_duration (2 s)
        cx = grid_x // 2
        top = max(0, cx - _OA_CORRIDOR_HALF)
        bot = min(grid_x, cx + _OA_CORRIDOR_HALF + 1)
        if not local_grid[top:bot, :].any():
            return

        with self.shared_data.mutex:
            if time() - self.shared_data.last_oa_trigger_time < _OA_COOLDOWN:
                return
            if not self.shared_data.gps_ready or not self.shared_data.new_waypoints:
                return
            goal = self.shared_data.new_waypoints[-1]
            path = self.shared_data.global_map.get_path(pos[0], pos[1], goal[0], goal[1])
            if not path:
                print("[OA] Corridor blocked but get_path returned empty — map not ready?")
                return
            if self.shared_data.obstacle_avoidance_running:
                print("[OA] Reattempt: replacing pending OA waypoints")
                self.shared_data.obstacle_avoidance_new_waypoints = []
                self.shared_data.new_obstacle_avoidance_waypoints_flag = False
            self.shared_data.obstacle_avoidance_new_waypoints = path
            self.shared_data.new_obstacle_avoidance_waypoints_flag = True
            self.shared_data.last_oa_trigger_time = time()
            print(f"[OA] Triggered from occupancy grid, path={len(path)} waypoints")

    def _tick(self):
        if self.shared_data.new_waypoints_flag:
            self.cur_state = self.state_instance.new_controller_state

        if self.last_state != self.cur_state:
            self.last_state.on_exit()
            self.last_state = self.cur_state
            self.cur_state.on_enter()
        else:
            self.last_state = self.cur_state
            self.cur_state = self.cur_state.step()

        if time() - self.shared_data.latest_ar_found > 2:
            self.shared_data.found_ar_flag = False

        self._map_print_counter += 1
        if self._map_print_counter >= RATE:
            self._map_print_counter = 0
            pos = self.shared_data.cur_position
            if pos is not None:
                path = self.shared_data.obstacle_avoidance_new_waypoints or None
                self.shared_data.global_map.print_ascii_map(pos[0], pos[1], path_waypoints=path)

    def setup_subscribers(self):
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE, depth=10)

        # Initialize subscribers for all relevant topics
        self.get_logger().info("[StartState] Initializing subscribers...")
        
        self.create_subscription(NavSatFix, self.shared_data.GPS_TOPIC_NAME, lambda msg: on_new_gps_msg(msg, self.shared_data), qos_profile)
        self.create_subscription(Float64, self.shared_data.COMPASS_TOPIC_NAME, lambda msg: on_new_heading_msg(msg, self.shared_data), qos_profile)
        self.create_subscription(Float64MultiArray, self.shared_data.WAYPOINTS_TOPIC_NAME, lambda msg: on_receive_new_gps_waypoint(msg, self.shared_data), 10)
        self.create_subscription(TwistStamped, self.shared_data.VELOCITY_TOPIC_NAME, lambda msg: on_new_velocity_msg(msg, self.shared_data), qos_profile)
        self.create_subscription(Float64MultiArray, self.shared_data.ARTAG_TOPIC_NAME, lambda msg: found_ar_tag(msg, self.shared_data), 10)
        self.create_subscription(Float64MultiArray, self.shared_data.OBJECT_TOPIC_NAME, lambda msg: found_object(msg, self.shared_data), 10)
        self.create_subscription(Float64MultiArray, self.shared_data.OBSTACLE_AVOIDANCE_WAYPOINTS_TOPIC_NAME, lambda msg: on_receive_new_obstacle_avoidance_waypoints(msg, self.shared_data), 10)
        self.create_subscription(Odometry, self.shared_data.LOCAL_ODOM_TOPIC, lambda msg: on_new_local_odom_msg(msg, self.shared_data), qos_profile)
        self.create_subscription(Odometry, self.shared_data.GLOBAL_ODOM_TOPIC, lambda msg: on_new_global_odom_msg(msg, self.shared_data), qos_profile)
        self.create_subscription(Float64MultiArray, '/obstacle_avoidance/occupancy_grid', self._on_occupancy_grid, 10)
        # self.create_subscription(Bool, self.shared_data.HEARTBEAT_TOPIC, lambda msg: heartbeat_reset(msg, self.shared_data), 10)
        self.get_logger().info("[StartState] Subscriber initialization complete.")

    def setup_publishers(self):
        self.get_logger().info("[StartState] Initializing publishers...")
        self.shared_data.auton_publisher = self.create_publisher(Float64MultiArray, self.shared_data.AUTON_TOPIC_NAME, 10)
        self.shared_data.cur_state_publisher = self.create_publisher(String, self.shared_data.CUR_STATE_TOPIC_NAME, 10)
        self.shared_data.auton_path_publisher = self.create_publisher(Float64MultiArray, self.shared_data.AUTON_PATH_NAME, 10)
        self.shared_data.led_publisher = self.create_publisher(Float64MultiArray, self.shared_data.LED_TOPIC_NAME, 10)
        self.get_logger().info("[StartState] Publisher initialization complete.")

    def build_states(self):
        self.state_instance.start_state = StartState(self.shared_data, self.state_instance)
        self.state_instance.new_controller_state = NewControllerState(self.shared_data, self.state_instance)
        self.state_instance.new_obstacle_avoidance_controller_state = NewObstacleAvoidanceControllerState(self.shared_data, self.state_instance)
        self.state_instance.drive_state = DriveState(self.shared_data, self.state_instance)
        self.state_instance.obstacle_recovery_state = ObstacleRecoveryState(self.shared_data, self.state_instance)
        self.state_instance.validating_state = ValidatingState(self.shared_data, self.state_instance)
        self.state_instance.goal_state = GoalState(self.shared_data, self.state_instance)
        self.state_instance.retry_state = RetryState(self.shared_data, self.state_instance)
        self.state_instance.spin_state = SpinState(self.shared_data, self.state_instance)
        self.state_instance.forward_spiral_state = ForwardSpiralState(self.shared_data, self.state_instance)
        self.state_instance.backward_spiral_state = BackwardSpiralState(self.shared_data, self.state_instance)
        self.state_instance.go_to_ar_state = GoToARState(self.shared_data, self.state_instance)
        self.state_instance.go_to_obj_state = GoToObjectState(self.shared_data, self.state_instance)
        self.state_instance.circle_state = CircleState(self.shared_data, self.state_instance)
        # self.state_instance.heartbeat_state = HeartbeatState(self.shared_data, self.state_instance)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
