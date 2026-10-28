from types import SimpleNamespace

from autonomous_state_machine import mission_clock


def test_mission_durations_follow_simulation_clock_and_hardware_uses_wall_time(monkeypatch):
    monkeypatch.setattr(mission_clock, "_clock", None)
    monkeypatch.setattr(mission_clock, "wall_time", lambda: 1234.0)
    assert mission_clock.time() == 1234.0
    mission_clock.bind_clock(SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=5_500_000_000)))
    assert mission_clock.time() == 5.5
    mission_clock.bind_clock(None)
    assert mission_clock.time() == 1234.0
