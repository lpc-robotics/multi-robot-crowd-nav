from types import SimpleNamespace
from unittest.mock import Mock
import threading
import time

import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from arena_multi_hunav.clock_gate import ClockStallGate
from arena_multi_hunav.multi_adapter import MultiSfmAdapter
from arena_multi_control.command_guard import CommandGuard
from arena_multi_hunav.demo import PsychologyDemo


ROBOTS = {"robot_1": (1_000_000_000, 1.0), "robot_2": (1_000_000_000, 1.0)}


def test_coherent_stall_requires_new_clock_and_every_new_robot():
    gate = ClockStallGate()
    assert gate.update(1.61, 1_000_000_000, 1.0, ROBOTS, .6) == (True, 0)
    one = {**ROBOTS, "robot_1": (1_050_000_000, 2.0)}
    assert gate.update(2.0, 1_050_000_000, 2.0, one, .6) == (True, 0)
    fresh = {n: (1_050_000_000, 2.05) for n in ROBOTS}
    hold, shifted = gate.update(2.05, 1_050_000_000, 2.0, fresh, .6)
    assert not hold and shifted == pytest.approx(.44)
    assert gate.recoveries == 1 and not gate.active


def test_active_clock_with_single_robot_loss_is_not_a_clock_stall():
    gate = ClockStallGate()
    robots = {**ROBOTS, "robot_1": (1_050_000_000, 1.61)}
    assert gate.update(1.61, 1_050_000_000, 1.61, robots, .6) == (False, 0)
    assert not gate.active


def test_clock_delivery_jitter_does_not_turn_coherent_pause_into_single_loss():
    gate = ClockStallGate()
    robots = {"robot_1": (1_000_000_000, 1.0), "robot_2": (1_000_000_000, 1.01)}
    assert gate.update(1.605, 1_000_000_000, 1.01, robots, .6)[0]


@pytest.mark.parametrize("stamp,now,error", [
    (900_000_000, 2.0, "clock_rewind"),
    (1_200_000_000, 2.0, "simulation_time_gap_after_stall"),
    (1_000_000_000, 31.01, "simulation_clock_stall_timeout"),
])
def test_rewind_jump_and_persistent_stall_remain_faults(stamp, now, error):
    gate = ClockStallGate()
    gate.update(1.61, 1_000_000_000, 1.0, ROBOTS, .6)
    with pytest.raises(ValueError, match=error):
        gate.update(now, stamp, 1.0, ROBOTS, .6)


def test_duplicate_robot_sample_cannot_release_hold():
    gate = ClockStallGate()
    gate.update(1.61, 1_000_000_000, 1.0, ROBOTS, .6)
    robots = {name: (1_000_000_000, 2.0) for name in ROBOTS}
    assert gate.update(2.0, 1_050_000_000, 2.0, robots, .6)[0]


def test_newer_but_already_expired_samples_cannot_release_hold():
    gate = ClockStallGate()
    gate.update(1.61, 1_000_000_000, 1.0, ROBOTS, .6)
    robots = {name: (1_050_000_000, 2.0) for name in ROBOTS}
    assert gate.update(3.0, 1_050_000_000, 2.0, robots, .6)[0]


def test_hold_does_not_read_or_commit_pending_transactions(monkeypatch):
    monkeypatch.setattr("arena_multi_hunav.multi_adapter.time.monotonic", lambda: 1.61)
    future = Mock()
    pending = (future, 1.0, object(), object(), object())
    state = SimpleNamespace(fault="", stamp_ns=1_000_000_000, clock_ns=1_000_000_000,
        last_clock_wall=1., clock_gate=ClockStallGate(), pending=pending,
        update_pending=(future, 1., 1_000_000_000, False), health_pub=Mock(),
        history={name: SimpleNamespace(samples=[SimpleNamespace(stamp_ns=s)], wall_seen=w)
                 for name, (s, w) in ROBOTS.items()},
        get_parameter=lambda name: SimpleNamespace(value=.6 if name=="observation_timeout" else 1.),
        get_logger=lambda: Mock(), _report=Mock())
    MultiSfmAdapter._tick_impl(state)
    assert state.clock_gate.active and state.pending is pending
    assert not future.done.called and not future.result.called
    assert state.health_pub.publish.call_args.args[0].data is False


@pytest.mark.parametrize("request_start, expected_start", [
    (.8, 2.8),  # Preserve the 0.2s wait before the stalled clock interval.
    (1.3, 3.0),  # Request sent before the 0.6s stall detection deadline.
    (3.1, 3.1),  # A subsequent request never receives extra time.
])
def test_request_deadline_excludes_full_confirmed_stall_overlap(request_start, expected_start):
    gate = ClockStallGate(clock_seen=1.0)
    assert gate.request_started_after_resume(request_start, 3.0) == pytest.approx(expected_start)


def test_recovery_requires_command_after_healthy_transition():
    old = Twist(); old.linear.x = .2
    state = SimpleNamespace(_lock=threading.RLock(), _command=old, _command_seen=1.,
        _command_valid=True, _pedestrian_healthy=True, require_pedestrian_health=True,
        mode="external", max_linear=.5, max_angular=1.)
    CommandGuard._pedestrian_health(state, Bool(data=False))
    CommandGuard._accept(state, "external", old)
    assert state._command_seen == 0 and state._command.linear.x == 0
    CommandGuard._pedestrian_health(state, Bool(data=True))
    assert state._command_seen == 0
    CommandGuard._accept(state, "external", old)
    assert state._command_seen > 0 and state._command.linear.x == .2


def test_demo_holds_existing_actions_during_fresh_pause_heartbeat():
    active = {"robot_1": {"handle": object()}}
    state = SimpleNamespace(status={"paused": True, "healthy": False},
        status_seen=time.monotonic(), active=active,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)))
    PsychologyDemo._advance(state)
    assert state.active is active
