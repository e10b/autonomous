"""Tests for the TF-retry-safety fix (Fix A) and the recalibration-triggers-replan
fix (Fix B) in autonomous_state_machine.py.

Follows the two existing patterns in this test suite:
  - SimpleNamespace shared-data + FakeLogger (test_replan.py)
  - calling the unbound AutonomousStateMachine method on a SimpleNamespace stand-in
    "node" (test_odom_watchdog.py)
"""
from threading import Lock
from time import time
from types import SimpleNamespace

from geometry_msgs.msg import TransformStamped

import autonomous_state_machine.config as config
from autonomous_state_machine.autonomous_state_machine import AutonomousStateMachine


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg, **kwargs):
        self.messages.append(("info", msg))

    def warn(self, msg, **kwargs):
        self.messages.append(("warn", msg))

    def error(self, msg, **kwargs):
        self.messages.append(("error", msg))


class FlakyTfBuffer:
    """Raises on the first `fail_times` lookups, then returns an identity transform."""

    def __init__(self, fail_times=0):
        self.fail_times = fail_times
        self.calls = 0

    def lookup_transform(self, target, source, time_):
        self.calls += 1
        if self.calls <= self.fail_times:
            raise LookupError("no transform yet")
        t = TransformStamped()
        t.transform.rotation.w = 1.0  # identity rotation, zero translation
        return t


def make_node(tf_buffer=None, **shared_overrides):
    shared_defaults = dict(
        mutex=Lock(),
        logger=FakeLogger(),
        local_odom_position=(1.0, 2.0),
        new_waypoints=[(5.0, 6.0)],
        new_waypoints_odom=[],
        cur_position=(1.0, 2.0),
    )
    shared_defaults.update(shared_overrides)
    shared = SimpleNamespace(**shared_defaults)
    logger = FakeLogger()
    node = SimpleNamespace(
        _tf_buffer=tf_buffer if tf_buffer is not None else FlakyTfBuffer(fail_times=0),
        shared_data=shared,
        get_logger=lambda: logger,
    )
    node._transform_goal_to_odom = lambda x, y: AutonomousStateMachine._transform_goal_to_odom(node, x, y)
    node._convert_waypoints_to_odom = lambda: AutonomousStateMachine._convert_waypoints_to_odom(node)
    return node, logger


# --- Fix A: _transform_goal_to_odom / _convert_waypoints_to_odom -----------------

def test_transform_goal_to_odom_returns_none_on_tf_failure():
    node, logger = make_node(tf_buffer=FlakyTfBuffer(fail_times=1))
    assert node._transform_goal_to_odom(5.0, 6.0) is None
    assert any(level == "warn" for level, _ in logger.messages)


def test_transform_goal_to_odom_succeeds_once_tf_becomes_available():
    tf = FlakyTfBuffer(fail_times=1)
    node, _ = make_node(tf_buffer=tf)
    assert node._transform_goal_to_odom(5.0, 6.0) is None          # first attempt: TF not ready
    assert node._transform_goal_to_odom(5.0, 6.0) == (5.0, 6.0)    # retried: succeeds (identity TF)


def test_convert_waypoints_to_odom_leaves_previous_path_untouched_on_tf_failure():
    node, _ = make_node(tf_buffer=FlakyTfBuffer(fail_times=5))
    node.shared_data.new_waypoints_odom = [(0.0, 0.0)]  # previous (good) path
    assert node._convert_waypoints_to_odom() is False
    assert node.shared_data.new_waypoints_odom == [(0.0, 0.0)]


def test_convert_waypoints_to_odom_rebuilds_path_once_tf_available():
    node, _ = make_node(tf_buffer=FlakyTfBuffer(fail_times=0))
    assert node._convert_waypoints_to_odom() is True
    assert node.shared_data.new_waypoints_odom == [(1.0, 2.0), (5.0, 6.0)]


def test_convert_waypoints_to_odom_returns_false_without_local_odom():
    node, _ = make_node(tf_buffer=FlakyTfBuffer(fail_times=0))
    node.shared_data.local_odom_position = None
    assert node._convert_waypoints_to_odom() is False


# --- Fix B: needs_recalibration must trigger a real replan, via a full _tick -----

class _FakeStateBase:
    """Stand-in for a State singleton; on_enter/on_exit/step are no-ops."""

    def __init__(self):
        self.entered = 0
        self.exited = 0

    def on_enter(self):
        self.entered += 1

    def on_exit(self):
        self.exited += 1

    def step(self):
        return self


def FakeState(name):
    """Build an instance of a distinct, uniquely-named class so _tick's
    `self.cur_state.__class__.__name__` logging matches the real state names
    (each call must get its own class — reusing one class and mutating
    __name__ would clobber every other instance's reported name too)."""
    return type(name, (_FakeStateBase,), {})()


def make_tick_node(cur_state_name="drive_state", **shared_overrides):
    states = SimpleNamespace(
        start_state=FakeState("StartState"),
        drive_state=FakeState("DriveState"),
        new_controller_state=FakeState("NewControllerState"),
        validating_state=FakeState("ValidatingState"),
    )
    shared_defaults = dict(
        mutex=Lock(),
        logger=FakeLogger(),
        last_odom_time=time(),
        local_odom_position=(1.0, 2.0),
        cur_position=(1.0, 2.0),
        goal_map_xy=(10.0, 20.0),
        goal_odom=(10.0, 20.0),
        goal_dirty=False,
        new_waypoints=[(10.0, 20.0)],
        new_waypoints_odom=[],
        new_waypoints_flag=False,
        needs_recalibration=False,
        recal_distance_accumulator=0.0,
        new_obstacle_avoidance_waypoints_flag=False,
        latest_ar_found=0, latest_obj_found=0,
        found_ar_flag=False, found_obj_flag=False, counter=0,
    )
    shared_defaults.update(shared_overrides)
    shared = SimpleNamespace(**shared_defaults)
    logger = FakeLogger()
    node = SimpleNamespace(
        _tf_buffer=FlakyTfBuffer(fail_times=0),
        shared_data=shared,
        state_instance=states,
        cur_state=getattr(states, cur_state_name),
        last_state=getattr(states, cur_state_name),
        _odom_stale=False,
        _map_print_counter=0,
        get_logger=lambda: logger,
    )
    node._handle_stale_odom = lambda: AutonomousStateMachine._handle_stale_odom(node)
    node._transform_goal_to_odom = lambda x, y: AutonomousStateMachine._transform_goal_to_odom(node, x, y)
    node._convert_waypoints_to_odom = lambda: AutonomousStateMachine._convert_waypoints_to_odom(node)
    return node, logger


def test_recalibration_triggers_replan_while_driving():
    node, _ = make_tick_node(needs_recalibration=True)
    AutonomousStateMachine._tick(node)
    # replan_to_goal + the now-TF-safe new_waypoints_flag path both ran this tick:
    # a fresh controller build was queued and completed (identity TF never fails here).
    assert node.shared_data.needs_recalibration is False
    assert node.shared_data.recal_distance_accumulator == 0.0
    assert node.shared_data.new_waypoints_odom == [(1.0, 2.0), (10.0, 20.0)]
    assert node.cur_state is node.state_instance.new_controller_state


def test_recalibration_does_not_fire_outside_drive_state():
    node, _ = make_tick_node(cur_state_name="validating_state", needs_recalibration=True)
    AutonomousStateMachine._tick(node)
    assert node.shared_data.needs_recalibration is True
    assert node.shared_data.new_waypoints_flag is False


def test_recalibration_does_not_clobber_a_pending_replan():
    node, _ = make_tick_node(needs_recalibration=True, new_waypoints_flag=True,
                              new_waypoints=[(99.0, 99.0)])
    AutonomousStateMachine._tick(node)
    # The in-flight replan (new_waypoints_flag, targeting (99, 99)) must win — recalibration's
    # own replan_to_goal() (which would overwrite new_waypoints with goal_map_xy=(10, 20))
    # must not have fired on top of it.
    assert node.shared_data.new_waypoints == [(99.0, 99.0)]


def test_recalibration_retries_next_tick_if_tf_unavailable():
    node, _ = make_tick_node(needs_recalibration=True)
    node._tf_buffer = FlakyTfBuffer(fail_times=100)  # TF never resolves this tick
    AutonomousStateMachine._tick(node)
    # replan_to_goal still queues new_waypoints_flag, but _convert_waypoints_to_odom
    # fails, so the state machine must NOT advance to NewControllerState, and
    # needs_recalibration/new_waypoints_flag must stay set so it's retried.
    assert node.cur_state is node.state_instance.drive_state
    assert node.shared_data.new_waypoints_flag is True
