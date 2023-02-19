import pytest
import state


def test_convert_arr_to_waypoint_list():
    input = [1, 2, 3, 4, 5, 6]
    expected_output = [(1, 2), (3, 4), (5, 6)]
    assert expected_output == state.convert_arr_to_waypoint_list(input)
