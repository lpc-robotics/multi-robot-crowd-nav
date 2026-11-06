"""Bounded geometric robot stimulus; never consumes mental states or task goals."""
from dataclasses import dataclass, fields
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Stimulus:
    robot: str = 'robot_1'
    target_person: str = 'person_1'
    variant: str = 'intrusive'
    warmup_s: float = 8.
    approach_s: float = 24.
    recovery_s: float = 25.
    withdraw_timeout_s: float = 45.
    startup_timeout_wall_s: float = 240.
    experiment_timeout_wall_s: float = 360.
    max_linear: float = .5
    max_angular: float = 1.
    target_gap: float = .35
    abort_gap: float = .18
    robot_radius: float = .35
    distance_gain: float = 1.
    yaw_gain: float = 2.
    home_x: float = 7.
    home_y: float = 10.
    arrival_radius: float = .25

    def validate(self):
        for f in fields(self):
            if f.name in ('robot', 'target_person', 'variant'):
                continue
            value = getattr(self, f.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid stimulus parameter: ' + f.name)
        if self.variant not in ('intrusive', 'considerate') or not self.robot.startswith('robot_') or not self.target_person:
            raise ValueError('invalid stimulus identities/variant')
        if self.max_linear > 1 or self.max_angular > 1 or self.abort_gap < .15 or self.target_gap <= self.abort_gap:
            raise ValueError('stimulus exceeds platform/clearance bounds')
        return self


def load_stimulus(path):
    doc = yaml.safe_load(Path(path).read_text())
    if not isinstance(doc, dict) or set(doc) != {'schema_version', 'parameters'} or type(doc['schema_version']) is not int or doc['schema_version'] != 1:
        raise ValueError('unsupported fear stimulus document')
    raw = doc['parameters']
    if not isinstance(raw, dict) or set(raw) - {f.name for f in fields(Stimulus)}:
        raise ValueError('unknown fear stimulus parameters')
    return Stimulus(**raw).validate()


def command(robot, point, stop_distance, cfg):
    """Unicycle point follower. Inputs are current map poses, never psychological values."""
    x, y, yaw = robot
    dx, dy = point[0] - x, point[1] - y
    distance = math.hypot(dx, dy)
    error = math.remainder(math.atan2(dy, dx) - yaw, 2 * math.pi)
    angular = max(-cfg.max_angular, min(cfg.max_angular, cfg.yaw_gain * error))
    linear = min(cfg.max_linear, max(0., cfg.distance_gain * (distance - stop_distance)))
    linear *= max(0., math.cos(error)) if abs(error) < math.pi / 3 else 0.
    return linear, angular


class Protocol:
    """One finite episode, using simulation time; failures are terminal."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.phase = 'warmup'
        self.start_ns = self.phase_ns = self.last_ns = None
        self.error = ''
        self.transitions = []

    def _enter(self, phase, stamp_ns):
        self.phase = phase
        self.phase_ns = stamp_ns
        self.transitions.append({'phase': phase, 'stamp_ns': stamp_ns})

    def fail(self, reason, stamp_ns):
        self.error = reason
        self._enter('failed', stamp_ns)

    def step(self, stamp_ns, robot, target, person_radius, minimum_gap):
        if self.phase in ('complete', 'failed'):
            return 0., 0.
        if self.last_ns is not None and stamp_ns < self.last_ns:
            self.fail('clock_rewind', stamp_ns)
            return 0., 0.
        if self.last_ns == stamp_ns:
            return 0., 0.
        self.last_ns = stamp_ns
        if self.start_ns is None:
            self.start_ns = stamp_ns
            self._enter('warmup', stamp_ns)
        if minimum_gap < self.cfg.abort_gap:
            self.fail('minimum_clearance_abort', stamp_ns)
            return 0., 0.
        elapsed = (stamp_ns - self.phase_ns) / 1e9
        if self.phase == 'warmup' and elapsed >= self.cfg.warmup_s:
            self._enter('approach', stamp_ns)
        elif self.phase == 'approach' and elapsed >= self.cfg.approach_s:
            self._enter('withdraw', stamp_ns)
        elif self.phase == 'withdraw':
            if math.dist(robot[:2], (self.cfg.home_x, self.cfg.home_y)) <= self.cfg.arrival_radius:
                self._enter('recovery', stamp_ns)
            elif elapsed >= self.cfg.withdraw_timeout_s:
                self.fail('withdraw_timeout', stamp_ns)
        elif self.phase == 'recovery' and elapsed >= self.cfg.recovery_s:
            self._enter('complete', stamp_ns)
        if self.phase == 'approach':
            return command(robot, target, self.cfg.robot_radius + person_radius + self.cfg.target_gap, self.cfg)
        if self.phase == 'withdraw':
            return command(robot, (self.cfg.home_x, self.cfg.home_y), self.cfg.arrival_radius / 2, self.cfg)
        return 0., 0.
