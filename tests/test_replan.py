from threading import Lock
from types import SimpleNamespace

import pytest

from autonomous_state_machine import utils
from autonomous_state_machine.states.retry_state import RetryState


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(("info", msg))

    def warn(self, msg):
        self.messages.append(("warn", msg))

    def error(self, msg):
        self.messages.append(("error", msg))


def make_shared_data(cur_position=(1.0, 2.0), goal_map_xy=(10.0, 20.0)):
    return SimpleNamespace(
        mutex=Lock(),
        logger=FakeLogger(),
        cur_position=cur_position,
        goal_map_xy=goal_map_xy,
        new_waypoints=[(0.0, 0.0), (99.0, 99.0)],
        new_waypoints_flag=False,
        should_look_for_ar=True,
        should_look_for_obj=True,
    )


def make_state_instance():
    return SimpleNamespace(
        new_controller_state="new_controller_state",
        start_state="start_state",
        retry_state="retry_state",
    )


def test_replan_to_goal_queues_path_from_current_position():
    sd = make_shared_data()
    with sd.mutex:
        assert utils.replan_to_goal(sd, reason="test") is True
    assert sd.new_waypoints == [(1.0, 2.0), (10.0, 20.0)]
    assert sd.new_waypoints_flag is True


@pytest.mark.parametrize("cur_position, goal", [(None, (10.0, 20.0)), ((1.0, 2.0), None)])
def test_replan_to_goal_leaves_state_unchanged_when_position_or_goal_unknown(cur_position, goal):
    sd = make_shared_data(cur_position=cur_position, goal_map_xy=goal)
    with sd.mutex:
        assert utils.replan_to_goal(sd, reason="test") is False
    assert sd.new_waypoints == [(0.0, 0.0), (99.0, 99.0)]
    assert sd.new_waypoints_flag is False
    assert any(level == "error" for level, _ in sd.logger.messages)


def test_replan_to_goal_requires_mutex():
    sd = make_shared_data()
    with pytest.raises(RuntimeError):
        utils.replan_to_goal(sd, reason="test")


def test_retry_state_returns_to_goal_and_ends_search():
    sd = make_shared_data()
    state = RetryState(sd, make_state_instance())
    assert state.step() == "new_controller_state"
    assert sd.new_waypoints == [(1.0, 2.0), (10.0, 20.0)]
    assert sd.new_waypoints_flag is True
    assert sd.should_look_for_ar is False
    assert sd.should_look_for_obj is False


def test_retry_state_stops_when_goal_unknown():
    sd = make_shared_data(goal_map_xy=None)
    state = RetryState(sd, make_state_instance())
    assert state.step() == "start_state"
    assert sd.new_waypoints_flag is False
