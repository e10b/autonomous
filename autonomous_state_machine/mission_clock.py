"""Use ROS simulation time for mission durations; wall time on hardware."""
from time import time as wall_time

_clock = None


def bind_clock(clock):
    global _clock
    _clock = clock


def time():
    return _clock.now().nanoseconds / 1e9 if _clock is not None else wall_time()
