#!/usr/bin/env python3

"""http://wiki.ros.org/mavros for topic ref"""

from autonomous_state_machine.utils import on_receive_new_auton_input, on_new_velocity_msg, found_ar_tag, found_object, on_new_local_odom_msg, on_heartbeat
import numpy as np
from scipy import ndimage
import rclpy
import rclpy.time
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TwistStamped, PointStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_msgs.msg import Float64MultiArray, String, Bool
from sensor_msgs.msg import PointCloud2
from time import time
from autonomous_state_machine.global_map import GlobalMapManager

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
from autonomous_state_machine.states.heartbeat_state import HeartbeatState
from autonomous_state_machine.shared_data import SharedData
import autonomous_state_machine.config as config

from custom_msg_types.msg import AutonInput

from robot_localization.srv import FromLL

RATE = 30

_OA_INTERRUPTIBLE = frozenset({
    "ForwardSpiralState", "BackwardSpiralState", "DriveState",
    "GoToARState", "GoToObjectState", "NewControllerState",
})

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

        # LL client for goal transofrm
        self.shared_data.llclient = self.create_client(FromLL, '/fromLL')
        while not self.shared_data.llclient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for FromLL service...')
        self.get_logger().info('FromLL service is available.')

    @staticmethod
    def _largest_connected_obstacle_size(binary_grid: np.ndarray) -> int:
        """Return the size in cells of the largest 4-connected obstacle blob."""
        if binary_grid.size == 0:
            return 0

        # ndimage.label's default 2D structuring element is a cross, i.e. 4-connectivity.
        labels, num_blobs = ndimage.label(binary_grid > 0)
        if num_blobs == 0:
            return 0

        # bincount[0] counts background cells; the rest are per-blob sizes.
        return int(np.bincount(labels.ravel())[1:].max())

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
            t = self._tf_buffer.lookup_transform('odom', 'map', rclpy.time.Time())
            out = tf2_geometry_msgs.do_transform_point(pt, t)
            self.get_logger().info(f"[TF] map→odom goal: ({out.point.x:.2f}, {out.point.y:.2f})")
            return (out.point.x, out.point.y)
        except Exception:
            self.get_logger().warn("[TF] map→odom transform failed — using map coords as odom fallback")
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
        corridor_grid = local_grid[top:bot, :]
        if not corridor_grid.any():
            return

        largest_obstacle = self._largest_connected_obstacle_size(corridor_grid)
        if largest_obstacle < config.MIN_OBSTACLE_SIZE:
            return

        with self.shared_data.mutex:
            if time() - self.shared_data.last_oa_trigger_time < config.OA_COOLDOWN_EXTRA + config.BACKUP_DURATION:
                return
            if not self.shared_data.odom_ready or not self.shared_data.new_waypoints:
                return
            goal = self.shared_data.goal_odom
            if goal is None:
                return

            spiral_states = (self.state_instance.forward_spiral_state, self.state_instance.backward_spiral_state)
            if self.shared_data.post_oa_state in spiral_states:
                # Spiral mode: Theta* to the first unblocked spiral waypoint, not the GPS goal
                spiral_pts = self.shared_data.spiral_waypoints
                oa_goal = None
                gm = self.shared_data.global_map
                if spiral_pts and gm.global_origin_x is not None:
                    binary_grid = (gm.grid > gm.obstacle_threshold).astype(np.uint8)
                    radius_cells = max(1, int(config.OA_ROBOT_RADIUS_M / gm.cell_size_m))
                    inflated = gm._minkowski_sum_grid(binary_grid, radius_cells)
                    for i, (wx, wy) in enumerate(spiral_pts):
                        sc = int((wx - gm.global_origin_x) / gm.cell_size_m)
                        sr = int((wy - gm.global_origin_y) / gm.cell_size_m)
                        if 0 <= sc < gm.grid_size and 0 <= sr < gm.grid_size and inflated[sc, sr] == 0:
                            oa_goal = (wx, wy)
                            self.shared_data.spiral_resume_index = i
                            break
                if oa_goal is None and spiral_pts:
                    # All blocked — navigate to nearest free point; no waypoints left to resume
                    oa_goal = gm.nearest_free_point(spiral_pts[-1][0], spiral_pts[-1][1], config.OA_ROBOT_RADIUS_M)
                    self.shared_data.spiral_resume_index = len(spiral_pts)
                    self.get_logger().warn(
                        f"[OA] All spiral waypoints blocked — redirecting to nearest free point "
                        f"({oa_goal[0]:.2f}, {oa_goal[1]:.2f})"
                    )
                if oa_goal is not None:
                    goal = oa_goal
            else:
                # GPS navigation mode: redirect if goal is inside obstacle/inflation zone
                adjusted_goal = self.shared_data.global_map.nearest_free_point(goal[0], goal[1], config.OA_ROBOT_RADIUS_M)
                if adjusted_goal[0] != goal[0] or adjusted_goal[1] != goal[1]:
                    self.get_logger().warn(
                        f"[OA] Goal ({goal[0]:.2f}, {goal[1]:.2f}) is inside an obstacle/inflation zone — "
                        f"redirecting to nearest free point ({adjusted_goal[0]:.2f}, {adjusted_goal[1]:.2f})"
                    )
                    goal = adjusted_goal

            path = self.shared_data.global_map.get_path(pos[0], pos[1], goal[0], goal[1], robot_radius_m=config.OA_ROBOT_RADIUS_M)
            if not path:
                self.get_logger().warn("[OA] Corridor blocked but Theta* returned no path — map may not be ready yet")
                return
            if self.shared_data.obstacle_avoidance_running:
                self.get_logger().info("[OA] Reattempt — replacing active OA waypoints with fresher path")
                self.shared_data.obstacle_avoidance_new_waypoints = []
                self.shared_data.new_obstacle_avoidance_waypoints_flag = False
            self.shared_data.obstacle_avoidance_new_waypoints = path
            self.shared_data.new_obstacle_avoidance_waypoints_flag = True
            self.shared_data.last_oa_trigger_time = time()
            self.get_logger().info(f"[OA] Obstacle detected — AlignAndMove path queued ({len(path)} waypoints)")
            if self.shared_data.oa_path_publisher is not None:
                self.shared_data.oa_path_publisher.publish(
                    self._waypoints_to_path_msg(path, self.get_clock().now().to_msg())
                )

    def _publish_occupancy_grid(self):
        gm = self.shared_data.global_map
        if gm.global_origin_x is None:
            return

        # Compute inflation halo so it can be encoded as a distinct cost value
        binary_grid = (gm.grid > gm.obstacle_threshold).astype(np.uint8)
        radius_cells = max(1, int(config.OA_ROBOT_RADIUS_M / gm.cell_size_m))
        inflated = gm._minkowski_sum_grid(binary_grid, radius_cells)

        # grid[c,r] (c=x, r=y) → row-major OccupancyGrid [r*width+c] via transpose+flatten
        flat = gm.grid.T.flatten().astype(np.int16)
        flat[flat == 50] = -1  # remap unknown → OccupancyGrid unknown (-1)

        # Overwrite inflation-halo cells with cost 70 so Foxglove renders them
        # in a distinct color between free (0) and obstacle (100)
        inflated_flat = inflated.T.flatten()
        binary_flat = binary_grid.T.flatten()
        inflation_halo = (inflated_flat == 1) & (binary_flat == 0)
        flat[inflation_halo] = 70

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = gm.cell_size_m
        msg.info.width = gm.grid_size
        msg.info.height = gm.grid_size
        msg.info.origin.position.x = gm.global_origin_x
        msg.info.origin.position.y = gm.global_origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = flat.astype(np.int8).tolist()
        self._occupancy_grid_pub.publish(msg)

    @staticmethod
    def _waypoints_to_path_msg(waypoints, stamp, frame_id='odom') -> Path:
        path_msg = Path()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = frame_id
        for (wx, wy) in waypoints:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        return path_msg

    def _tick(self):
        old_state = self.cur_state.__class__.__name__

        # Re-transform goal to odom only when new waypoints arrived (dirty flag = O(1))
        if self.shared_data.goal_map_xy is not None and self.shared_data.goal_dirty:
            self.shared_data.goal_odom = self._transform_goal_to_odom(*self.shared_data.goal_map_xy)
            self.shared_data.goal_dirty = False

        if self.shared_data.needs_recalibration and self.shared_data.new_waypoints:
            self._convert_waypoints_to_odom()
            self.shared_data.needs_recalibration = False
            self.get_logger().info(f"Refreshed odom-frame waypoints after {config.RECALIBRATION_DISTANCE}m travel")

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
                    self.get_logger().info(f"[DriveState] goal reached ({d:.2f}m) — entering ValidatingState")
                    # Mark PP controller reached so ValidatingState sees did_reach_goal()=True.
                    # Without this, a freshly-built controller has reached_goal=False and loops:
                    # _tick→validating→new_controller→drive→_tick→...
                    if self.shared_data.controller is not None:
                        self.shared_data.controller.reached_goal = True
                    self.cur_state = self.state_instance.validating_state

        if config.HEARTBEAT_ENABLED:
            with self.shared_data.mutex:
                hb_time   = self.shared_data.last_heartbeat_time
                hb_pos    = self.shared_data.last_safe_position
                hb_active = self.shared_data.heartbeat_recovery_active
            # Update last_safe_position only while idle at a known safe checkpoint
            if (self.cur_state in (self.state_instance.start_state,
                                   self.state_instance.goal_state)
                    and hb_time > 0
                    and time() - hb_time < config.HEARTBEAT_TIMEOUT):
                with self.shared_data.mutex:
                    if self.shared_data.local_odom_position is not None:
                        self.shared_data.last_safe_position = self.shared_data.local_odom_position
            # Dropout check — runs after safe position update so dropout can't overwrite it
            if (hb_time > 0
                    and time() - hb_time > config.HEARTBEAT_TIMEOUT
                    and not hb_active
                    and self.cur_state is not self.state_instance.start_state):
                self.get_logger().warn("[Heartbeat] Signal lost — forcing return to last safe position")
                with self.shared_data.mutex:
                    cur = self.shared_data.local_odom_position
                    if cur is not None and hb_pos is not None:
                        self.shared_data.new_waypoints_odom = [cur, hb_pos]
                    self.shared_data.heartbeat_recovery_active = True
                self.cur_state = self.state_instance.heartbeat_state

        if self.shared_data.new_waypoints_flag:
            self._convert_waypoints_to_odom()
            self.shared_data.recal_distance_accumulator = 0.0
            self.shared_data.needs_recalibration = False
            self.cur_state = self.state_instance.new_controller_state

        if (self.shared_data.new_obstacle_avoidance_waypoints_flag
                and not self.shared_data.obstacle_avoidance_running
                and not self.shared_data.new_waypoints_flag
                and old_state in _OA_INTERRUPTIBLE):
            self.shared_data.obstacle_avoidance_running = True
            self.cur_state = self.state_instance.new_obstacle_avoidance_controller_state

        if self.last_state != self.cur_state:
            self.last_state.on_exit()
            self.last_state = self.cur_state
            self.cur_state.on_enter()
        else:
            self.last_state = self.cur_state
            self.cur_state = self.cur_state.step()

        if time() - self.shared_data.latest_ar_found > 2:
            self.shared_data.found_ar_flag = False
        if time() - self.shared_data.latest_obj_found > 2:
            # Object lost: clear the flag and require 2 fresh detections again
            self.shared_data.found_obj_flag = False
            self.shared_data.counter = 0

        if config.WRITE_ASCII_MAP or config.PUBLISH_COSTMAP:
            self._map_print_counter += 1
            if self._map_print_counter >= RATE:
                self._map_print_counter = 0
                pos = self.shared_data.local_odom_position
                if pos is not None:
                    if config.WRITE_ASCII_MAP:
                        path = self.shared_data.obstacle_avoidance_new_waypoints or None
                        self.shared_data.global_map.write_ascii_map(
                            config.ASCII_MAP_PATH, pos[0], pos[1],
                            path_waypoints=path, goal=self.shared_data.goal_odom)
                    if config.PUBLISH_COSTMAP:
                        self._publish_occupancy_grid()
                elif config.WRITE_ASCII_MAP:
                    try:
                        with open(config.ASCII_MAP_PATH, "w") as f:
                            f.write("--- Autonomous State Machine: waiting for pose data ---\n")
                    except OSError as e:
                        self.get_logger().warn(f"could not write ASCII map to {config.ASCII_MAP_PATH}: {e}")
        
        if config.HEARTBEAT_ENABLED:
            recovery_msg = Bool()
            recovery_msg.data = self.shared_data.heartbeat_recovery_active
            self.shared_data.heartbeat_recovery_publisher.publish(recovery_msg)

        new_state = self.cur_state.__class__.__name__
        if old_state != new_state:
            self.get_logger().info(f"[Transition] {old_state} → {new_state}")

    def setup_subscribers(self):
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE, depth=10)

        # Initialize subscribers for all relevant topics
        self.get_logger().info("[StartState] Initializing subscribers...")
        
        self.create_subscription(AutonInput, self.shared_data.AUTON_INPUT_TOPIC_NAME, lambda msg: on_receive_new_auton_input(msg, self.shared_data), 10)
        self.create_subscription(TwistStamped, self.shared_data.VELOCITY_TOPIC_NAME, lambda msg: on_new_velocity_msg(msg, self.shared_data), qos_profile)
        self.create_subscription(Float64MultiArray, self.shared_data.ARTAG_TOPIC_NAME, lambda msg: found_ar_tag(msg, self.shared_data), 10)
        self.create_subscription(Float64MultiArray, self.shared_data.OBJECT_TOPIC_NAME, lambda msg: found_object(msg, self.shared_data), 10)
        self.create_subscription(Odometry, self.shared_data.LOCAL_ODOM_TOPIC, lambda msg: on_new_local_odom_msg(msg, self.shared_data), qos_profile)
        self.create_subscription(PointCloud2, '/autonomous/obstacle_avoidance/oakd/pointcloud', self._on_pointcloud, qos_profile)
        if config.HEARTBEAT_ENABLED:
            self.create_subscription(Bool, self.shared_data.HEARTBEAT_TOPIC_NAME, lambda msg: on_heartbeat(msg, self.shared_data), 10)
        self.get_logger().info("[StartState] Subscriber initialization complete.")

    def setup_publishers(self):
        self.get_logger().info("[StartState] Initializing publishers...")
        self.shared_data.auton_publisher = self.create_publisher(Twist, self.shared_data.AUTON_TOPIC_NAME, 10)
        self.shared_data.cur_state_publisher = self.create_publisher(String, self.shared_data.CUR_STATE_TOPIC_NAME, 10)
        self.shared_data.auton_path_publisher = self.create_publisher(Float64MultiArray, self.shared_data.AUTON_PATH_NAME, 10)
        self.shared_data.led_publisher = self.create_publisher(Float64MultiArray, self.shared_data.LED_TOPIC_NAME, 10)
        if config.HEARTBEAT_ENABLED:
            self.shared_data.heartbeat_recovery_publisher = self.create_publisher(Bool, self.shared_data.HEARTBEAT_RECOVERY_TOPIC_NAME, 10)
        if config.PUBLISH_COSTMAP:
            self._occupancy_grid_pub = self.create_publisher(OccupancyGrid, '/autonomous/global_map', 1)
            self.shared_data.spiral_path_publisher = self.create_publisher(Path, '/autonomous/viz/spiral_path', 1)
            self.shared_data.oa_path_publisher = self.create_publisher(Path, '/autonomous/viz/oa_path', 1)
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
        self.state_instance.heartbeat_state = HeartbeatState(self.shared_data, self.state_instance)


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
