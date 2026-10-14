"""Strict versioned configuration shared by launch and the pure model."""
from dataclasses import asdict, dataclass, fields
import hashlib
import json
import math
from pathlib import Path
import yaml


@dataclass(frozen=True)
class PsychologyConfig:
    alpha_C: float = .60
    alpha_F: float = 1.00
    beta_C: float = .06
    beta_F: float = .12
    mu_C: float = .12
    mu_F: float = .25
    kappa: float = 0.
    novelty_decay: float = .05
    expressivity_C: float = .8
    expressivity_F: float = .8
    robot_visual_range: float = 8.
    spread_range: float = 3.
    robot_fov_deg: float = 200.
    human_fov_deg: float = 200.
    robot_fov_fade_deg: float = 10.
    human_fov_fade_deg: float = 10.
    prediction_horizon: float = 2.
    gap_warn: float = .8
    gap_critical: float = .15
    predicted_gap_warn: float = .8
    closing_speed_ref: float = .5
    curiosity_on: float = .6
    curiosity_off: float = .35
    fear_on: float = .65
    fear_off: float = .35
    vigilance_on: float = .08
    vigilance_off: float = .04
    curiosity_max_fear: float = .25
    curiosity_max_risk: float = .25
    emergency_risk: float = .85
    min_mode_hold: float = .5
    target_switch_margin: float = .15
    target_min_hold: float = 1.
    memory_ttl: float = 2.
    memory_tau: float = .8
    memory_prediction_horizon: float = .75
    minimum_memory_confidence: float = .2
    curiosity_standoff_gap: float = .9
    curiosity_stop_band: float = .15
    curiosity_speed: float = .35
    curiosity_gap_gain: float = .7
    curiosity_retreat_cap: float = .2
    scared_target_gap: float = 1.5
    scared_speed: float = .8
    scared_gap_gain: float = .8
    vigilant_speed_gain: float = .4
    human_visual_range: float = 5.
    approach_clearance: float = .35
    approach_retry_delay: float = 2.

    @classmethod
    def parse(cls, document):
        if not isinstance(document, dict) or set(document) != {"schema_version", "model", "occlusion_model", "parameters"}:
            raise ValueError("psychology config requires schema_version/model/occlusion_model/parameters")
        if type(document["schema_version"]) is not int or document["schema_version"] != 1 or document["model"] != "sis_cf_v1":
            raise ValueError("unsupported psychology config version/model")
        if document["occlusion_model"] != "none_open_space":
            raise ValueError("only none_open_space perception is implemented")
        raw = document["parameters"]
        known = {f.name for f in fields(cls)}
        if not isinstance(raw, dict) or set(raw) - known:
            raise ValueError("unknown psychology parameters")
        values = {}
        for name, value in raw.items():
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0:
                raise ValueError("invalid psychology parameter: " + name)
            values[name] = float(value)
        result = cls(**values)
        result.validate()
        return result

    def validate(self):
        for name, value in asdict(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError("invalid psychology parameter: " + name)
        for name in ("mu_C", "mu_F", "robot_visual_range", "human_visual_range", "spread_range", "prediction_horizon", "closing_speed_ref", "memory_tau"):
            if getattr(self, name) <= 0:
                raise ValueError(name + " must be positive")
        for stem in ("curiosity", "fear", "vigilance"):
            if not 0 <= getattr(self, stem + "_off") < getattr(self, stem + "_on") <= 1:
                raise ValueError("invalid hysteresis: " + stem)
        for name in ("expressivity_C", "expressivity_F", "curiosity_max_fear", "curiosity_max_risk", "emergency_risk", "minimum_memory_confidence", "vigilant_speed_gain"):
            if getattr(self, name) > 1:
                raise ValueError(name + " outside [0,1]")
        for stem in ("robot", "human"):
            angle, fade = getattr(self, stem + "_fov_deg"), getattr(self, stem + "_fov_fade_deg")
            if not 0 < angle <= 360 or not 0 <= fade <= angle / 2:
                raise ValueError("invalid " + stem + " FOV")
        if min(self.gap_warn, self.predicted_gap_warn) <= self.gap_critical:
            raise ValueError("risk warning gap must exceed critical gap")
        if self.memory_prediction_horizon > self.memory_ttl:
            raise ValueError("memory prediction exceeds TTL")
        if max(self.curiosity_speed, self.curiosity_retreat_cap, self.scared_speed) > 1:
            raise ValueError("behavior speed exceeds physical cap")

    def document(self):
        return {"schema_version": 1, "model": "sis_cf_v1", "occlusion_model": "none_open_space", "parameters": asdict(self)}

    @property
    def digest(self):
        return hashlib.sha256(json.dumps(self.document(), sort_keys=True).encode()).hexdigest()


def load_psychology_config(path):
    return PsychologyConfig.parse(yaml.safe_load(Path(path).read_text(encoding="utf-8")))
