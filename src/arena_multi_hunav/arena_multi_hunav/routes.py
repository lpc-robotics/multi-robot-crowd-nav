"""Compile configured routes into the existing cyclic SFM goal queue."""
from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from collections.abc import Mapping


@dataclass(frozen=True)
class Route:
    names: tuple[str, ...]
    points: tuple[tuple[float, float], ...]
    cyclic: bool


def point(value, *, person, target):
    try:
        x, y = float(value["x"]), float(value["y"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{person}.{target}: requires numeric x/y") from error
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError(f"{person}.{target}: coordinates must be finite")
    return x, y


def check_bounds(position, radius, world_bounds, *, person, target):
    if world_bounds is None:
        return
    try:
        width, height = map(float, world_bounds)
    except (ValueError, TypeError) as error:
        raise ValueError("world_bounds must contain width and height") from error
    if not all(math.isfinite(v) and v > 0 for v in (width, height)):
        raise ValueError("world_bounds must be finite and positive")
    x, y = position
    if not (radius <= x <= width - radius and radius <= y <= height - radius):
        raise ValueError(f"{person}.{target}: outside world bounds with radius {radius}")


def compile_route(spec, *, person, world_bounds=None):
    """Leave legacy queues intact; expand explicit ping-pong routes once."""
    keys = spec.get("goals")
    if not isinstance(keys, list) or any(not isinstance(k, str) for k in keys):
        raise ValueError(f"{person}.goals: expected a list of goal names")
    explicit = "route_mode" in spec
    mode = spec.get("route_mode")
    if explicit:
        if mode not in ("loop", "ping_pong"):
            raise ValueError(f"{person}.route_mode: expected loop or ping_pong")
        if len(keys) < 2:
            raise ValueError(f"{person}.goals: route requires at least two goals")
        if len(set(keys)) != len(keys):
            raise ValueError(f"{person}.goals: duplicate goal reference")
        if "cyclic_goals" in spec and spec["cyclic_goals"] is not True:
            raise ValueError(f"{person}.cyclic_goals: conflicts with route_mode {mode}")
        cyclic = True
    else:
        cyclic = bool(spec["cyclic_goals"])
    try:
        radius, goal_radius = float(spec["radius"]), float(spec["goal_radius"])
    except (KeyError, ValueError, TypeError) as error:
        raise ValueError(f"{person}: invalid radius or goal_radius") from error
    if not all(math.isfinite(v) and v > 0 for v in (radius, goal_radius)):
        raise ValueError(f"{person}: radius and goal_radius must be finite and positive")
    # Agent stores these as float32 on the wire. Validate the same envelope the
    # C++ service will see, including values exactly on a wall/arrival boundary.
    try:
        radius, goal_radius = struct.unpack("ff", struct.pack("ff", radius, goal_radius))
    except (OverflowError, struct.error) as error:
        raise ValueError(f"{person}: radius or goal_radius is outside float32 range") from error
    if not all(math.isfinite(v) and v > 0 for v in (radius, goal_radius)):
        raise ValueError(f"{person}: radius and goal_radius must be finite and positive in float32")
    points = []
    for key in keys:
        if key not in spec or not isinstance(spec[key], Mapping):
            raise ValueError(f"{person}.{key}: missing goal definition")
        xy = point(spec[key], person=person, target=key)
        check_bounds(xy, radius, world_bounds, person=person, target=key)
        points.append(xy)
    if explicit and len(set(points)) != len(points):
        raise ValueError(f"{person}.goals: duplicate goal coordinates")
    if mode == "ping_pong":
        keys = keys + keys[-2:0:-1]
        points = points + points[-2:0:-1]
    if explicit:
        for index, first in enumerate(points):
            following = (index + 1) % len(points)
            if math.dist(first, points[following]) <= 2 * goal_radius:
                raise ValueError(
                    f"{person}.{keys[index]}/{keys[following]}: goal arrival regions overlap")
    initial = point(spec.get("init_pose"), person=person, target="init_pose")
    check_bounds(initial, radius, world_bounds, person=person, target="init_pose")
    return Route(tuple(keys), tuple(points), cyclic)
