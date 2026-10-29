"""Versioned intent; legacy mode numbers and behavior module remain unchanged."""
from dataclasses import dataclass, asdict
import math
from .behavior import Intent, MODES

MODES_V2 = {**MODES, "SURPRISED": 3, "THREATENING": 4}


@dataclass(frozen=True)
class IntentV3(Intent):
    robot_response: int = 0  # 0 dynamic social, 1 circular static obstacle
    override_yaw: bool = False
    desired_yaw: float = 0.
    max_yaw_rate: float = 1.
    yaw_stop_speed: float = .05

    @classmethod
    def from_legacy(cls, intent, **extra):
        return cls(**asdict(intent), **extra)

    def validate(self):
        if self.mode not in MODES_V2:
            raise ValueError("invalid V3 mode")
        values = (self.desired_velocity_x, self.desired_velocity_y, self.desired_yaw,
                  self.max_yaw_rate, self.yaw_stop_speed)
        if not all(math.isfinite(v) for v in values) or math.hypot(*values[:2]) > 1 + 1e-9:
            raise ValueError("invalid V3 motion")
        if self.has_target_robot != (self.target_robot_id < 0) or (not self.has_target_robot and self.target_robot_id != 0):
            raise ValueError("invalid V3 target")
        if self.robot_response not in (0, 1) or not 0 < self.max_yaw_rate <= 1 or not 0 <= self.yaw_stop_speed <= .05:
            raise ValueError("invalid V3 response/orientation bounds")
        if self.mode == 'REGULAR':
            if self.has_target_robot or not self.allow_task_progress or self.override_yaw:
                raise ValueError("invalid V3 regular intent")
        elif not self.has_target_robot or not self.override_desired_velocity or self.allow_task_progress:
            raise ValueError("invalid V3 temporary intent")
        if not self.override_desired_velocity and any(values[:2]):
            raise ValueError("inactive desired velocity")
