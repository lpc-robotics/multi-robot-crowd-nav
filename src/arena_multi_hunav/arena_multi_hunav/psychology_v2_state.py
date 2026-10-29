"""All event state belongs to the proposed mental transaction, never a wall timer."""
from dataclasses import dataclass, field, replace
import math
from .psychology_sis import CrowdMentalState
from .behavior_v2 import MODES_V2


@dataclass
class CrowdMentalStateV2(CrowdMentalState):
    surprise: float = 0.
    last_stamp_ns: int = 0
    visible_since_ns: dict[int, int] = field(default_factory=dict)
    surprise_consumed: list[int] = field(default_factory=list)
    surprise_completed: list[int] = field(default_factory=list)
    aligned_observation_s: float = 0.
    event: str = ""
    event_target_robot_id: int = 0
    exit_reason: str = ""
    curious_target: int = 0
    curious_since_ns: int = 0
    phase: str = "none"
    phase_since_ns: int = 0
    interaction_target: int = 0
    target_point: tuple | None = None
    robot_heading: float = 0.
    episode_counts: dict[int, int] = field(default_factory=dict)
    cooldown_until_ns: dict[int, int] = field(default_factory=dict)
    playful_returning: bool = False
    playful_outcome: str = ""
    playful_blockers: list[str] = field(default_factory=list)

    def validate(self):
        # Validate the shared geometry/activation contract without teaching V1 new modes.
        CrowdMentalState(self.variables, self.compartments, replace(self.behavior, mode='REGULAR'),
                         self.memories, self.neighbors).validate()
        if self.behavior.mode not in MODES_V2 or not math.isfinite(self.surprise) or not 0 <= self.surprise <= 1:
            raise ValueError("invalid V2 state/mode")
        if type(self.event_target_robot_id) is not int or self.event_target_robot_id > 0:
            raise ValueError('invalid event target')
        if not math.isfinite(self.aligned_observation_s) or self.aligned_observation_s < 0 or self.last_stamp_ns < 0:
            raise ValueError("invalid V2 time")
        for ids in (self.surprise_consumed, self.surprise_completed):
            if len(ids) != len(set(ids)) or any(type(i) is not int or i >= 0 for i in ids):
                raise ValueError("invalid surprise identity")
        if not set(self.surprise_completed) <= set(self.surprise_consumed):
            raise ValueError("unconsumed completed surprise")
        if any(type(i) is not int or i >= 0 or type(t) is not int or t < 0 or t > self.last_stamp_ns
               for i, t in self.visible_since_ns.items()):
            raise ValueError("invalid continuous observation timer")
        if self.phase not in ('none', 'intercept', 'occupy', 'yield'):
            raise ValueError('invalid playful phase')
        if min(self.curious_since_ns, self.phase_since_ns) < 0 or max(self.curious_since_ns, self.phase_since_ns) > self.last_stamp_ns:
            raise ValueError('invalid playful timer')
        if self.phase != 'none' and (self.interaction_target >= 0 or self.target_point is None):
            raise ValueError('active interaction has no target')
        if self.target_point is not None and (len(self.target_point) != 2 or not all(math.isfinite(v) for v in self.target_point)):
            raise ValueError('invalid interaction point')
        if not math.isfinite(self.robot_heading):
            raise ValueError('invalid interaction heading')
        for records in (self.episode_counts, self.cooldown_until_ns):
            if any(type(i) is not int or i >= 0 or type(v) is not int or v < 0 for i,v in records.items()):
                raise ValueError('invalid episode budget')
