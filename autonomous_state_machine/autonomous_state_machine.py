#!/usr/bin/env python3

"""http://wiki.ros.org/mavros for topic ref"""

from autonomous_state_machine.utils import on_new_gps_msg, on_new_heading_msg, on_receive_new_gps_waypoint, on_new_velocity_msg, found_ar_tag, found_object, on_receive_new_obstacle_avoidance_waypoints, on_new_local_odom_msg
import numpy as np
import rclpy
import rclpy.time
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TwistStamped, PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import NavSatFix, PointCloud2, Imu
from time import time
from autonomous_state_machine.global_map import GlobalMapManager

from autonomous_state_machine.states.state_class import State
from autonomous_state_machine.states.state_class import StateInstance

from autonomous_state_machine.states.start_state import StartState
from autonomous_state_machine.states.forward_spiral_state import ForwardSpiralState
from autonomous_state_machine.states.goal_state import GoalState
from autonomous_state_machine.states.new_controller_state import NewControllerState
from autonomous_state_machine.states.retry_state import RetryState
from autonomous_state_machine.states.new_obstacle_avoidance_controller_state import NewObstacleAvoidanceControllerState
from autonomous_state_machine.states.drive_state import DriveState
from autonomous_state_machine.states.validating_state import ValidatingState
from autonomous_state_machine.states.go_to_object_state import GoToObjectState
from autonomous_state_machine.states.spin_state import SpinState
from autonomous_state_machine.states.backward_spiral_state import BackwardSpiralState
from autonomous_state_machine.states.go_to_ar_state import GoToARState
from autonomous_state_machine.states.circle_state import CircleState
from autonomous_state_machine.shared_data import SharedData
import autonomous_state_machine.config as config

RATE = 30

class AutonomousStateMachine(Node):
    def __init__(self):
        super().__init__("autonomous_state_machine")
        self.get_logger().info("Starting autonomous state machine node...")

        self.shared_data = SharedData()  # the global state object
        self.state_instance = StateInstance()

        self.shared_data.node = self
        self.shared_data.logger = self.get_logger()
        self.shared_data.global_map = GlobalMapManager(
            map_size_m=config.PERSISTENT_GRID_SIZE,
            overlap_m=config.OVERLAP_SIZE,
            shift_m=config.SHIFT_SIZE,
            cell_size_m=config.GRID_CELL_SIZE,
            free_space_decay=config.CONFIDENCE_DEGRADED_AMOUNT,
            obstacle_confidence_increase=config.CONFIDENCE_ADDED_AMOUNT,
            obstacle_threshold=config.CONFIDENCE_THRESHOLD,
            logger=self.get_logger()
        )
        self.build_states()
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.setup_subscribers()
        self.setup_publishers()

        self.cur_state = self.state_instance.start_state
        self.last_state = self.cur_state
        self.cur_state.on_enter()

        self._map_print_counter = 0
        self.timer = self.create_timer(1.0 / RATE, self._tick)


    @staticmethod
    def _parse_pointcloud2_xyz(msg: PointCloud2) -> np.ndarray:
        """
        Returns an (N, 3) float32 array of valid (x, y, z) points from a PointCloud2.
        Parses the raw byte buffer directly for performance — assumes x, y, z are
        FLOAT32 fields (datatype=7), which is always true for depth_image_proc output.
        """
        n = msg.width * msg.height
        step = msg.point_step
        if n == 0 or step == 0 or step % 4 != 0:
            return np.empty((0, 3), dtype=np.float32)

        # Interpret the entire data buffer as a float32 array and reshape to (N, floats_per_point)
        raw = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(n, step // 4)

        # Locate the float32 column index for each field using its byte offset
        col = {f.name: f.offset // 4 for f in msg.fields if f.datatype == 7}
        x = raw[:, col['x']]
        y = raw[:, col['y']]
        z = raw[:, col['z']]

        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        return np.stack([x[valid], y[valid], z[valid]], axis=1)

    def _transform_goal_to_odom(self, x_map: float, y_map: float):
        pt = PointStamped()
        pt.header.frame_id = 'map'
        pt.header.stamp = rclpy.time.Time().to_msg()  # zero stamp = latest TF
        pt.point.x = float(x_map)
        pt.point.y = float(y_map)
        try:
            print("TRANSFORM ATTEMPT: Transforming goal from map to odom frame...")
            t = self._tf_buffer.lookup_transform('odom', 'map', rclpy.time.Time())
            out = tf2_geometry_msgs.do_transform_point(pt, t)
            print("TRANSFORM SUCCESS: Goal in odom frame is ({:.2f}, {:.2f})".format(out.point.x, out.point.y))
            return (out.point.x, out.point.y)
        except Exception:
            print("TRANSFORM ERROR:Failed to transform goal to odom frame")
            return (x_map, y_map)  # fallback: in sim map≡odom so values are identical

    def _convert_waypoints_to_odom(self):
        """Convert new_waypoints (map/GPS-NED frame) to odom frame and store in new_waypoints_odom."""
        odom_pos = self.shared_data.local_odom_position
        if odom_pos is None:
            return
        odom_wps = [odom_pos]   # [0] is always the current odom position
        for wp in self.shared_data.new_waypoints[1:]:
            converted = self._transform_goal_to_odom(wp[0], wp[1])
            odom_wps.append(converted)
        self.shared_data.new_waypoints_odom = odom_wps

    def _on_pointcloud(self, msg: PointCloud2):
        pos = self.shared_data.local_odom_position
        yaw = self.shared_data.cur_heading
        if pos is None or yaw is None:
            return

        self.shared_data.global_map.initialize_origin(pos[0], pos[1])

        CELLS = config.PC_LOCAL_GRID_CELLS
        CELL  = config.GRID_CELL_SIZE
        half  = CELLS / 2.0

        pts = self._parse_pointcloud2_xyz(msg)
        if len(pts) == 0:
            return

        cam_x, cam_y, cam_z = pts[:, 0], pts[:, 1], pts[:, 2]

        # OAK-D optical frame: x=right, y=down, z=forward.
        # Keep only points in the obstacle height band and within the 2m forward grid.
        mask = (
            (cam_y >= config.PC_HEIGHT_MIN) & (cam_y <= config.PC_HEIGHT_MAX) &
            (cam_z >= 0.0)                 & (cam_z < CELLS * CELL)          &
            (cam_x >= -half * CELL)        & (cam_x <  half * CELL)
        )
        cam_x = cam_x[mask]
        cam_z = cam_z[mask]

        xi = np.clip((cam_x / CELL + half).astype(np.int32), 0, CELLS - 1)
        zi = np.clip((cam_z / CELL).astype(np.int32),        0, CELLS - 1)

        point_count = np.zeros((CELLS, CELLS), dtype=np.int32)
        np.add.at(point_count, (xi, zi), 1)
        local_grid = (point_count >= config.PC_MIN_POINTS_PER_CELL).astype(np.uint8)

        self.shared_data.global_map.update_map(local_grid, pos[0], pos[1], yaw)

        cx  = CELLS // 2
        top = max(0, cx - config.OA_CORRIDOR_HALF_CELLS)
        bot = min(CELLS, cx + config.OA_CORRIDOR_HALF_CELLS + 1)
        if not local_grid[top:bot, :].any():
            return

        with self.shared_data.mutex:
            if time() - self.shared_data.last_oa_trigger_time < config.OA_COOLDOWN_EXTRA + config.BACKUP_DURATION:
                return
            if not self.shared_data.odom_ready or not self.shared_data.new_waypoints:
                return
            goal = self.shared_data.goal_odom
            if goal is None:
                return
            path = self.shared_data.global_map.get_path(pos[0], pos[1], goal[0], goal[1], robot_radius_m=config.OA_ROBOT_RADIUS_M)
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
            print(f"[OA] Triggered from pointcloud, path={len(path)} waypoints")

    def _tick(self):
        # Re-transform goal to odom only when new waypoints arrived (dirty flag = O(1))
        if self.shared_data.goal_map_xy is not None and self.shared_data.goal_dirty:
            self.shared_data.goal_odom = self._transform_goal_to_odom(*self.shared_data.goal_map_xy)
            self.shared_data.goal_dirty = False

        # Every-tick goal detection in odom frame (mirrors Navigation::remain() in C++)
        if self.cur_state is self.state_instance.drive_state:
            odom_pos = self.shared_data.local_odom_position
            goal     = self.shared_data.goal_odom
            if odom_pos is None or goal is None:
                self._goal_skip_counter = getattr(self, '_goal_skip_counter', 0) + 1
                if self._goal_skip_counter >= RATE:
                    self._goal_skip_counter = 0
                    print(f"[GoalCheck] waiting — odom={'SET' if odom_pos else 'NONE'} goal_odom={'SET' if goal else 'NONE'}")
            else:
                self._goal_skip_counter = 0
                d = np.linalg.norm(np.array(odom_pos) - np.array(goal))
                if d < 2.0:
                    print(f"[GoalCheck] GOAL REACHED ({d:.2f}m) — entering validating_state")
                    # Mark PP controller reached so ValidatingState sees did_reach_goal()=True.
                    # Without this, a freshly-built controller has reached_goal=False and loops:
                    # _tick→validating→new_controller→drive→_tick→...
                    if self.shared_data.controller is not None:
                        self.shared_data.controller.reached_goal = True
                    self.cur_state = self.state_instance.validating_state

        if self.shared_data.new_waypoints_flag:
            self._convert_waypoints_to_odom()
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

        if config.PRINT_ASCII_MAP:
            self._map_print_counter += 1
            if self._map_print_counter >= RATE:
                self._map_print_counter = 0
                pos = self.shared_data.local_odom_position
                if pos is not None:
                    path = self.shared_data.obstacle_avoidance_new_waypoints or None
                    self.shared_data.global_map.print_ascii_map(pos[0], pos[1], path_waypoints=path, goal=self.shared_data.goal_odom)
                else:
                    try:
                        with open("/tmp/rover_map.txt", "w") as f:
                            f.write("--- Autonomous State Machine: waiting for pose data ---\n")
                    except Exception:
                        pass

    def setup_subscribers(self):
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE, depth=10)

        # Initialize subscribers for all relevant topics
        self.get_logger().info("[StartState] Initializing subscribers...")
        
        self.create_subscription(NavSatFix, self.shared_data.GPS_TOPIC_NAME, lambda msg: on_new_gps_msg(msg, self.shared_data), qos_profile)
        self.create_subscription(Imu, self.shared_data.COMPASS_TOPIC_NAME, lambda msg: on_new_heading_msg(msg, self.shared_data), qos_profile)
        self.create_subscription(Float64MultiArray, self.shared_data.WAYPOINTS_TOPIC_NAME, lambda msg: on_receive_new_gps_waypoint(msg, self.shared_data), 10)
        self.create_subscription(TwistStamped, self.shared_data.VELOCITY_TOPIC_NAME, lambda msg: on_new_velocity_msg(msg, self.shared_data), qos_profile)
        self.create_subscription(Float64MultiArray, self.shared_data.ARTAG_TOPIC_NAME, lambda msg: found_ar_tag(msg, self.shared_data), 10)
        self.create_subscription(Float64MultiArray, self.shared_data.OBJECT_TOPIC_NAME, lambda msg: found_object(msg, self.shared_data), 10)
        self.create_subscription(Float64MultiArray, self.shared_data.OBSTACLE_AVOIDANCE_WAYPOINTS_TOPIC_NAME, lambda msg: on_receive_new_obstacle_avoidance_waypoints(msg, self.shared_data), 10)
        self.create_subscription(Odometry, self.shared_data.LOCAL_ODOM_TOPIC, lambda msg: on_new_local_odom_msg(msg, self.shared_data), qos_profile)
        self.create_subscription(PointCloud2, '/autonomous/obstacle_avoidance/oakd/pointcloud', self._on_pointcloud, qos_profile)
        self.get_logger().info("[StartState] Subscriber initialization complete.")

    def setup_publishers(self):
        self.get_logger().info("[StartState] Initializing publishers...")
        self.shared_data.auton_publisher = self.create_publisher(Twist, self.shared_data.AUTON_TOPIC_NAME, 10)
        self.shared_data.cur_state_publisher = self.create_publisher(String, self.shared_data.CUR_STATE_TOPIC_NAME, 10)
        self.shared_data.auton_path_publisher = self.create_publisher(Float64MultiArray, self.shared_data.AUTON_PATH_NAME, 10)
        self.shared_data.led_publisher = self.create_publisher(Float64MultiArray, self.shared_data.LED_TOPIC_NAME, 10)
        self.get_logger().info("[StartState] Publisher initialization complete.")

    def build_states(self):
        self.state_instance.start_state = StartState(self.shared_data, self.state_instance)
        self.state_instance.new_controller_state = NewControllerState(self.shared_data, self.state_instance)
        self.state_instance.new_obstacle_avoidance_controller_state = NewObstacleAvoidanceControllerState(self.shared_data, self.state_instance)
        self.state_instance.drive_state = DriveState(self.shared_data, self.state_instance)
        self.state_instance.validating_state = ValidatingState(self.shared_data, self.state_instance)
        self.state_instance.goal_state = GoalState(self.shared_data, self.state_instance)
        self.state_instance.retry_state = RetryState(self.shared_data, self.state_instance)
        self.state_instance.spin_state = SpinState(self.shared_data, self.state_instance)
        self.state_instance.forward_spiral_state = ForwardSpiralState(self.shared_data, self.state_instance)
        self.state_instance.backward_spiral_state = BackwardSpiralState(self.shared_data, self.state_instance)
        self.state_instance.go_to_ar_state = GoToARState(self.shared_data, self.state_instance)
        self.state_instance.go_to_obj_state = GoToObjectState(self.shared_data, self.state_instance)
        self.state_instance.circle_state = CircleState(self.shared_data, self.state_instance)


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
