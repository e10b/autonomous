"""
Planner node — your implementation goes here.

Run via:
    python sim/launch.py                       # default scenario
    python sim/launch.py --scenario open       # easiest world — start here
    python sim/launch.py --visualize           # with a live plot
    python sim/launch.py --all                 # every scenario, one summary table

This file uses rclpy_lite, a lightweight simulator shim that mirrors the real
rclpy (ROS 2 Python) API. The class names, method signatures, and message types
are identical to real ROS 2 — only the import paths differ:

    Real ROS 2:     import rclpy / from rclpy.node import Node
    This shim:      import rclpy_lite as rclpy / from rclpy_lite.node import Node

To run on a real ROS 2 system, swap the imports back to plain rclpy.

Division of labour
------------------
    grid_utils.py   grid maths (world <-> cell, inflating obstacles)     no ROS
    search.py       A*                                                    no ROS
    replan.py       "is my plan still valid? where am I on it?"           no ROS
    planner_node.py THIS FILE: receives grids, decides when to replan,
                    publishes paths, prints monitoring output              ROS glue

Get the three pure files passing their tests first. Then this file is mostly wiring.
"""

import rclpy_lite as rclpy
from rclpy_lite.node import Node
from rclpy_lite.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Message types provided by the simulator
from sim.messages import GridSnapshot, make_path_msg

# Standard ROS message type (mirrored by the shim)
from nav_msgs.msg import Path

from grid_utils import Grid, inflate, path_length_m
from sim.grid_values import OCCUPIED
from search import astar
from replan import next_waypoint_index, path_is_valid


class PlannerNode(Node):
    """
    Plans a path from the rover to the goal and keeps it up to date as the map changes.

    Subscribes:
        /grid_feed     (sim.messages.GridSnapshot)   5 Hz, BEST_EFFORT
                       occupancy grid + rover position + goal, all in one message

    Publishes:
        /planned_path  (nav_msgs/Path)               only when the plan changes
                       waypoints in WORLD coordinates; an EMPTY path means "stop"
    """

    def __init__(self):
        super().__init__("planner_node")

        # ------------------------------------------------------------------
        # Subscriber
        # ------------------------------------------------------------------
        # TODO: Create a subscription to /grid_feed that calls self.on_grid.
        #
        # IMPORTANT: check the QoS the feed publishes with (see
        # sim/grid_feed_publisher.py). If your subscription asks for more than
        # the publisher offers, ROS connects nothing and you receive nothing.
        # The sim prints a warning when this happens — read the terminal output.
        #
        # self.grid_sub = self.create_subscription(
        #     GridSnapshot, "/grid_feed", self.on_grid, ???
        # )
        feed_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.grid_sub = self.create_subscription(
            GridSnapshot, "/grid_feed", self.on_grid, feed_qos
        )

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        # TODO: Create a publisher for /planned_path (message type: Path).
        #
        # self.path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)

        # ------------------------------------------------------------------
        # State — add whatever you need
        # ------------------------------------------------------------------
        self.path_xy = None          # current plan: list of (x, y) WORLD points. None = never planned.
        self.plan_count = 0          # how many plans you have published (initial one included)
        self.replan_count = 0        # how many of those replaced an existing plan
        self.last_replan_stamp = None  # sim time of the most recent replan, or None
        self.last_status_stamp = None
        self._last_no_route_blocked = None
        self._pending_status_reason = None

    # -----------------------------------------------------------------------
    # The callback: runs once per tick
    # -----------------------------------------------------------------------

    def on_grid(self, msg: GridSnapshot) -> None:
        """
        Called every tick (5 Hz of SIMULATED time) with the rover's latest view.

        Your job on each call:
          1. Turn the message into a Grid (Grid.from_msg) and make it safe for a
             rover with a body (inflate by msg.robot_radius_m).
          2. Work out where the rover and the goal are in the grid.
          3. Decide: is the current plan still good?  (self.needs_replan)
          4. Only if it isn't: plan again (self.plan), publish it (self.publish_path),
             and update your counters.
          5. Print monitoring output (self.print_status).

        Step 3 is the heart of the task: replan when the plan is invalidated,
        and not otherwise. The scoreboard tells you how you did.
        """
        grid = inflate(Grid.from_msg(msg), msg.robot_radius_m)
        self._block_unsafe_edge_cells(grid, msg.robot_radius_m)
        rover_xy = (msg.rover_x, msg.rover_y)
        goal_xy = (msg.goal_x, msg.goal_y)
        should_plan, reason = self.needs_replan(grid, rover_xy)

        if should_plan:
            replacing_path = bool(self.path_xy)
            self.path_xy = self.plan(grid, rover_xy, goal_xy)
            self.publish_path(self.path_xy, msg.header.stamp)
            self.plan_count += 1
            if replacing_path:
                self.replan_count += 1
                self.last_replan_stamp = msg.header.stamp
            self._last_no_route_blocked = (
                frozenset(i for i, value in enumerate(grid.occupancy) if value == OCCUPIED)
                if not self.path_xy else None
            )
            self._pending_status_reason = reason

        self.print_status(msg.header.stamp)

    @staticmethod
    def _block_unsafe_edge_cells(grid: Grid, radius_m: float) -> None:
        """Treat the map boundary as a wall with the same clearance as obstacles."""
        for row in range(grid.height):
            for col in range(grid.width):
                x_clearance = min(
                    (col + 0.5) * grid.resolution,
                    (grid.width - col - 0.5) * grid.resolution,
                )
                y_clearance = min(
                    (row + 0.5) * grid.resolution,
                    (grid.height - row - 0.5) * grid.resolution,
                )
                if min(x_clearance, y_clearance) < radius_m - 1e-12:
                    grid.occupancy[grid.index(row, col)] = OCCUPIED

    def needs_replan(self, grid: Grid, rover_xy) -> tuple:
        """
        Should we compute a new plan right now?  Returns (bool, reason_string).

        `grid` is the current (already inflated) grid. `rover_xy` is where the rover is now.

        Think about every situation in which the answer should be True:
          * you have never planned
          * the last plan found no route (was it worth trying again? when?)
          * the plan is no longer safe
        and the situations in which it should be False, even though the map changed:
          * something changed, but not anywhere near the part of the path still ahead

        Return a short reason like "path blocked ahead" — it goes in your monitoring
        output and makes debugging a hundred times easier.
        """
        if self.path_xy is None:
            return True, "initial plan"
        if not self.path_xy:
            # Unknown cells are already traversable, so only clearing something
            # that was blocked can make an impossible route possible. New
            # obstacles elsewhere are not a reason to republish another stop.
            route_may_have_opened = any(
                grid.occupancy[i] != OCCUPIED for i in self._last_no_route_blocked
            )
            return ((True, "blocked cell cleared") if route_may_have_opened
                    else (False, "no route unchanged"))

        current = next_waypoint_index(self.path_xy, rover_xy)
        if not path_is_valid(grid, self.path_xy, current):
            return True, "path blocked ahead"
        return False, "path still valid"

    def plan(self, grid: Grid, rover_xy, goal_xy):
        """
        Run the search and return the result as a list of (x, y) WORLD points
        (first = the rover's cell, last = the goal's cell), or [] if there is no route.

        Convert world -> cell, call astar, convert the cells back to world points.
        """
        start = grid.world_to_cell(*rover_xy)
        goal = grid.world_to_cell(*goal_xy)
        cells = astar(grid, start, goal, unknown_is_free=True)
        if cells is None:
            return []
        return [grid.cell_to_world(*cell) for cell in cells]

    def publish_path(self, points_xy, stamp: float) -> None:
        """Publish a list of (x, y) world points as nav_msgs/Path. (provided)"""
        self.path_pub.publish(make_path_msg(points_xy, stamp))

    # -----------------------------------------------------------------------
    # Monitoring
    # -----------------------------------------------------------------------

    def print_status(self, stamp: float) -> None:
        """
        Print a monitoring line to the terminal. Not every tick — about once per
        second of simulated time is plenty, plus a line whenever you (re)plan.

        Must show:
          * current path length (metres)
          * number of replans triggered so far
          * time since the last replan

        `stamp` is SIMULATION time in seconds (msg.header.stamp). The sim can run
        much faster than real time, so time.time() would give nonsense here.
        """
        due = self.last_status_stamp is None or stamp - self.last_status_stamp >= 1.0 - 1e-9
        if self._pending_status_reason is None and not due:
            return

        length = path_length_m(self.path_xy or [])
        since = "n/a" if self.last_replan_stamp is None else f"{stamp - self.last_replan_stamp:.1f}s"
        reason = f" reason={self._pending_status_reason}" if self._pending_status_reason else ""
        self.get_logger().info(
            f"t={stamp:.1f}s path={length:.1f}m replans={self.replan_count} "
            f"since_replan={since}{reason}"
        )
        self.last_status_stamp = stamp
        self._pending_status_reason = None
