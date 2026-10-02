from threading import Lock
from time import time
from types import SimpleNamespace

from nav_msgs.msg import Odometry

import autonomous_state_machine.config as config
from autonomous_state_machine import utils
from autonomous_state_machine.autonomous_state_machine import AutonomousStateMachine


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg, **kwargs):
        self.messages.append(("info", msg))

    def warn(self, msg, **kwargs):
        self.messages.append(("warn", msg))


class FakePublisher:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append((msg.linear.x, msg.angular.z))


class FakeState:
    def __init__(self):
        self.steps = 0

    def step(self):
        self.steps += 1
        return self


def make_node(cur_state_name="drive_state", last_odom_time=None):
    """Minimal stand-in for AutonomousStateMachine with just what the watchdog touches."""
    states = SimpleNamespace(start_state=FakeState(), drive_state=FakeState())
    logger = FakeLogger()
    node = SimpleNamespace(
        state_instance=states,
        cur_state=getattr(states, cur_state_name),
        shared_data=SimpleNamespace(
            last_odom_time=time() if last_odom_time is None else last_odom_time,
            auton_publisher=FakePublisher(),
        ),
        _odom_stale=False,
        get_logger=lambda: logger,
    )
    node._handle_stale_odom = lambda: AutonomousStateMachine._handle_stale_odom(node)
    return node, logger


def stale_time():
    return time() - (config.ODOM_TIMEOUT + 1.0)


def test_fresh_odom_does_not_stop():
    node, logger = make_node()
    assert node._handle_stale_odom() is False
    assert node.shared_data.auton_publisher.msgs == []
    assert logger.messages == []


def test_stale_odom_stops_every_tick_and_warns_once():
    node, logger = make_node(last_odom_time=stale_time())
    for _ in range(3):
        assert node._handle_stale_odom() is True
    assert node.shared_data.auton_publisher.msgs == [(0.0, 0.0)] * 3
    assert [level for level, _ in logger.messages] == ["warn"]


def test_never_received_odom_counts_as_stale():
    node, _ = make_node(last_odom_time=0.0)
    assert node._handle_stale_odom() is True


def test_start_state_is_exempt():
    node, logger = make_node(cur_state_name="start_state", last_odom_time=0.0)
    assert node._handle_stale_odom() is False
    assert node.shared_data.auton_publisher.msgs == []
    assert logger.messages == []


def test_resumes_and_logs_when_odom_returns():
    node, logger = make_node(last_odom_time=stale_time())
    assert node._handle_stale_odom() is True
    node.shared_data.last_odom_time = time()
    assert node._handle_stale_odom() is False
    assert node._odom_stale is False
    assert [level for level, _ in logger.messages] == ["warn", "info"]


def test_tick_pauses_state_machine_while_stale():
    node, _ = make_node(last_odom_time=stale_time())
    AutonomousStateMachine._tick(node)
    assert node.cur_state.steps == 0


def test_odom_callback_records_arrival_time():
    shared_data = SimpleNamespace(
        mutex=Lock(), last_odom_time=0.0, _prev_local_odom_pos=None, recal_distance_accumulator=0.0,
        needs_recalibration=False, new_waypoints=[], local_odom_position=None, last_safe_position=None,
        odom_ready=False, cur_position=None, position_history=[], cur_heading=None,
    )
    before = time()
    utils.on_new_local_odom_msg(Odometry(), shared_data)
    assert before <= shared_data.last_odom_time <= time()
