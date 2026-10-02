import math
import pytest
from autonomous_state_machine import utils


def test_convert_arr_to_waypoint_list():
    input = [1, 2, 3, 4, 5, 6]
    expected_output = [(1, 2), (3, 4), (5, 6)]
    assert utils.convert_arr_to_waypoint_list(input) == expected_output


def test_rotate_and_translate_2d_translation():
    pos = (3, 4)
    coord = (4, 5)
    heading = math.pi / 2
    expected = (7, 9)
    actual = utils.rotate_and_translate_2d(*coord, *pos, heading)
    assert pytest.approx(actual) == expected


def test_rotate_and_translate_2d_no_rotation():
    pos = (0, 0)
    coord = (4, 5)
    heading = math.pi / 2
    expected = (4, 5)
    actual = utils.rotate_and_translate_2d(*coord, *pos, heading)
    assert pytest.approx(actual) == expected


def test_rotate_and_translate_2d_some_rotation():
    pos = (0, 0)
    coord = (4, 5)
    heading = 0
    expected = (5, -4)
    actual = utils.rotate_and_translate_2d(*coord, *pos, heading)
    assert pytest.approx(actual) == expected


def test_rotate_and_translate_2d_rotation_and_translate():
    pos = (6, 2)
    coord = (-5 * math.sqrt(2) / 2, math.sqrt(2) / 2)
    heading = math.pi / 4
    expected = (4, 5)
    actual = utils.rotate_and_translate_2d(*coord, *pos, heading)
    assert pytest.approx(actual) == expected
