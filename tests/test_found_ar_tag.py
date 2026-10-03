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


def make_shared_data(target_id=0):
    # Defaults match SharedData and the values left behind by reset_ar_data()
    return SimpleNamespace(
        mutex=Lock(), logger=FakeLogger(), target_id=target_id,
        found_ar_flag=False, latest_ar_found=0,
        ar_cX=0, ar_cY=0, ar_size=0, ar_id=0,
    )


def ar_msg(cX, cY, size, tag_id):
    return Float64MultiArray(data=[float(cX), float(cY), float(size), float(tag_id)])


def test_non_matching_tag_does_not_set_found_flag():
    # Previously any tag set the flag when the stored ar_id == target_id (both 0 by default)
    sd = make_shared_data(target_id=0)
    utils.found_ar_tag(ar_msg(0.7, 0.4, 0.05, 3), sd)
    assert sd.found_ar_flag is False
    assert sd.latest_ar_found == 0
    assert (sd.ar_cX, sd.ar_cY, sd.ar_size, sd.ar_id) == (0, 0, 0, 0)


def test_non_matching_tag_after_a_match_does_not_refresh_detection(monkeypatch):
    monkeypatch.setattr(utils, "time", lambda: 10.0)
    sd = make_shared_data(target_id=2)
    utils.found_ar_tag(ar_msg(0.6, 0.5, 0.04, 2), sd)
    monkeypatch.setattr(utils, "time", lambda: 20.0)
    utils.found_ar_tag(ar_msg(0.1, 0.1, 0.09, 4), sd)
    assert sd.latest_ar_found == 10.0
    assert (sd.ar_cX, sd.ar_cY, sd.ar_size, sd.ar_id) == (0.6, 0.5, 0.04, 2.0)


def test_matching_tag_sets_flag_fields_and_timestamp(monkeypatch):
    monkeypatch.setattr(utils, "time", lambda: 123.0)
    sd = make_shared_data(target_id=2)
    utils.found_ar_tag(ar_msg(0.6, 0.5, 0.04, 2), sd)
    assert sd.found_ar_flag is True
    assert sd.latest_ar_found == 123.0
    assert (sd.ar_cX, sd.ar_cY, sd.ar_size, sd.ar_id) == (0.6, 0.5, 0.04, 2.0)


def test_mutex_released_after_callback():
    sd = make_shared_data(target_id=1)
    for tag_id in (1, 3):
        utils.found_ar_tag(ar_msg(0.5, 0.5, 0.01, tag_id), sd)
        assert not sd.mutex.locked()


@pytest.mark.parametrize("data", [[], [0.5, 0.5, 0.0], [0.5, 0.5, 0.01, 0.0, 9.0]])
def test_malformed_message_is_ignored_and_mutex_released(data):
    sd = make_shared_data(target_id=0)
    utils.found_ar_tag(Float64MultiArray(data=data), sd)
    assert not sd.mutex.locked()
    assert sd.found_ar_flag is False
    assert (sd.ar_cX, sd.ar_cY, sd.ar_size, sd.ar_id) == (0, 0, 0, 0)
    assert [level for level, _ in sd.logger.messages] == ["warn"]
