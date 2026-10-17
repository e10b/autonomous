from threading import Lock
from types import SimpleNamespace

import pytest
from std_msgs.msg import Float64MultiArray

from autonomous_state_machine import utils


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg, **kwargs):
        self.messages.append(("info", msg))

    def warn(self, msg, **kwargs):
        self.messages.append(("warn", msg))


def make_shared_data(target_obj_id=1.0):
    return SimpleNamespace(
        mutex=Lock(), logger=FakeLogger(), target_obj_id=target_obj_id,
        counter=0, found_obj_flag=False, latest_obj_found=0,
        obj_cX=0, obj_cY=0, obj_width=0, obj_height=0, obj_id=0,
    )


def obj_msg(cX, cY, w, h, obj_id):
    return Float64MultiArray(data=[float(cX), float(cY), float(w), float(h), float(obj_id)])


def test_flag_set_after_second_matching_detection(monkeypatch):
    monkeypatch.setattr(utils, "time", lambda: 50.0)
    sd = make_shared_data(target_obj_id=1.0)
    utils.found_object(obj_msg(0.4, 0.5, 0.03, 0.04, 1), sd)
    assert sd.counter == 1
    assert sd.found_obj_flag is False
    utils.found_object(obj_msg(0.45, 0.5, 0.03, 0.04, 1), sd)
    assert sd.counter == 2
    assert sd.found_obj_flag is True
    assert sd.latest_obj_found == 50.0
    assert (sd.obj_cX, sd.obj_cY, sd.obj_width, sd.obj_height, sd.obj_id) == (0.45, 0.5, 0.03, 0.04, 1.0)
    assert not sd.mutex.locked()


def test_non_matching_object_changes_nothing():
    sd = make_shared_data(target_obj_id=1.0)
    utils.found_object(obj_msg(0.4, 0.5, 0.03, 0.04, 3), sd)
    assert sd.counter == 0
    assert sd.found_obj_flag is False
    assert (sd.obj_cX, sd.obj_cY, sd.obj_width, sd.obj_height, sd.obj_id) == (0, 0, 0, 0, 0)
    assert not sd.mutex.locked()


@pytest.mark.parametrize("data", [[], [0.4, 0.5, 1.0], [0.4, 0.5, 0.03, 0.04, 0.0, 1.0]])
def test_malformed_message_is_ignored_and_mutex_released(data):
    sd = make_shared_data(target_obj_id=1.0)
    utils.found_object(Float64MultiArray(data=data), sd)
    assert not sd.mutex.locked()
    assert sd.counter == 0
    assert sd.found_obj_flag is False
    assert (sd.obj_cX, sd.obj_cY, sd.obj_width, sd.obj_height, sd.obj_id) == (0, 0, 0, 0, 0)
    assert [level for level, _ in sd.logger.messages] == ["warn"]
