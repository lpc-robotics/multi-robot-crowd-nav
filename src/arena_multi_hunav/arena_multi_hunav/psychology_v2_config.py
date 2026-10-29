"""V2 configuration. Parameter values are engineering initial values, not fitted."""
from dataclasses import asdict, dataclass, field, fields, replace
import hashlib
import json
import math

from .psychology_config import PsychologyConfig

PROFILES = ("standard", "immune_dynamic", "impassive_static", "playful")


@dataclass(frozen=True)
class ParametersV2(PsychologyConfig):
    surprise_visible_s: float = .3
    surprise_novelty: float = .6
    surprise_max_risk: float = .25
    surprise_tau: float = 1.5
    surprise_off: float = .25
    observation_stop_speed: float = .05
    observation_yaw_rate: float = 1.
    observation_angle_deg: float = 15.
    observation_hold_s: float = 1.
    observation_display_margin_s: float = .15
    surprise_timeout_s: float = 4.
    playful_curiosity: float = .70
    playful_max_fear: float = .20
    playful_max_risk: float = .25
    playful_curious_s: float = 4.
    playful_front_gap: float = 1.
    playful_lookahead_s: float = 1.
    playful_intercept_s: float = 4.
    playful_occupy_s: float = 2.
    playful_yield_s: float = 3.
    playful_cooldown_s: float = 30.
    playful_max_episodes: float = 1.
    playful_min_robot_speed: float = .05
    playful_turn_limit_deg: float = 45.
    playful_speed: float = .6
    playful_arrival_radius: float = .25
    playful_yield_distance: float = 1.2
    # Experimental ablation; not a latent psychological parameter.
    playful_enabled: float = 1.
    # Open-world geometry, must match the physical scenario.
    world_width: float = 30.
    world_height: float = 23.

    @classmethod
    def from_values(cls, values):
        known = {f.name for f in fields(cls)}
        if not isinstance(values, dict) or set(values) - known:
            raise ValueError("unknown V2 parameters")
        if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v)
               for v in values.values()):
            raise ValueError("invalid V2 numeric parameter")
        result = cls(**{k: float(v) for k, v in values.items()})
        result.validate()
        return result

    def validate(self):
        super().validate()
        for key in ("surprise_visible_s", "surprise_tau", "observation_yaw_rate", "observation_hold_s",
                    "surprise_timeout_s", "playful_intercept_s", "playful_occupy_s", "playful_yield_s",
                    "playful_min_robot_speed", "playful_arrival_radius", "playful_yield_distance",
                    "world_width", "world_height"):
            if getattr(self, key) <= 0:
                raise ValueError(key + " must be positive")
        for key in ("surprise_novelty", "surprise_max_risk", "surprise_off", "playful_curiosity",
                    "playful_max_fear", "playful_max_risk", "playful_enabled"):
            if not 0 <= getattr(self, key) <= 1:
                raise ValueError(key + " outside [0,1]")
        if self.playful_enabled not in (0., 1.) or self.playful_max_episodes != int(self.playful_max_episodes):
            raise ValueError("invalid playful switch/count")
        if not 0 < self.surprise_off < 1 or self.surprise_timeout_s < self.observation_hold_s:
            raise ValueError("invalid surprise completion bounds")
        if not 0 < self.observation_angle_deg <= 180 or not 0 < self.playful_turn_limit_deg <= 180:
            raise ValueError("invalid V2 angle")
        if self.observation_stop_speed > .05 or self.observation_yaw_rate > 1 or self.playful_speed > 1:
            raise ValueError("V2 control exceeds physical/display contract")


@dataclass(frozen=True)
class Individual:
    profile: str = "standard"
    preferred_robot_id: int = 0
    parameters: dict = field(default_factory=dict)


@dataclass(frozen=True)
class PsychologyConfigV2:
    parameters: ParametersV2
    individuals: dict[int, Individual]

    @classmethod
    def parse(cls, doc):
        if not isinstance(doc, dict) or set(doc) != {"schema_version", "model", "occlusion_model", "parameters", "individuals"}:
            raise ValueError("V2 requires schema_version/model/occlusion_model/parameters/individuals")
        if type(doc['schema_version']) is not int or doc['schema_version'] != 2 or doc['model'] != 'sis_cf_v2':
            raise ValueError("unsupported V2 version/model")
        if doc['occlusion_model'] != 'none_open_space':
            raise ValueError("V2 only supports the open-space world")
        base = ParametersV2.from_values(doc['parameters'])
        if not isinstance(doc['individuals'], dict):
            raise ValueError("individuals must be an id mapping")
        people = {}
        for key, raw in doc['individuals'].items():
            if type(key) is not int or key <= 0 or not isinstance(raw, dict) or set(raw) - {'profile', 'preferred_robot_id', 'parameters'}:
                raise ValueError("invalid V2 individual")
            item = Individual(**raw)
            if item.profile not in PROFILES or type(item.preferred_robot_id) is not int or item.preferred_robot_id > 0:
                raise ValueError("invalid profile/preferred robot")
            if not isinstance(item.parameters, dict):
                raise ValueError("individual parameters must be a mapping")
            if set(item.parameters) & {'world_width', 'world_height'}:
                raise ValueError("world geometry cannot be an individual override")
            ParametersV2.from_values({**asdict(base), **item.parameters})
            people[key] = item
        return cls(base, people)

    def resolved(self, agent_id):
        person = self.individuals.get(agent_id, Individual())
        return person, ParametersV2.from_values({**asdict(self.parameters), **person.parameters})

    def document(self):
        return dict(schema_version=2, model='sis_cf_v2', occlusion_model='none_open_space',
                    parameters=asdict(self.parameters), individuals={i: asdict(p) for i, p in self.individuals.items()})

    @property
    def digest(self):
        return hashlib.sha256(json.dumps(self.document(), sort_keys=True).encode()).hexdigest()
