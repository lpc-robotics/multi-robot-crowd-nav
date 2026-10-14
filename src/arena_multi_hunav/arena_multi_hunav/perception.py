"""Geometric perception; no occlusion or hidden-state target inference."""
import math


def clip01(value):
    return min(1., max(0., value))


def visibility(me, other, distance_limit, fov, fade):
    dx, dy = other.x - me.x, other.y - me.y
    if math.hypot(dx, dy) >= distance_limit:
        return 0.
    if fov == 360 or (dx == 0 and dy == 0):
        return 1.
    angle = abs(math.degrees(math.atan2(-dx * math.sin(me.yaw) + dy * math.cos(me.yaw),
                                      dx * math.cos(me.yaw) + dy * math.sin(me.yaw))))
    return float(angle <= fov / 2) if fade == 0 else clip01((fov / 2 - angle) / fade)


def risk(me, other, cfg):
    dx, dy = other.x - me.x, other.y - me.y
    ux, uy = other.vx - me.vx, other.vy - me.vy
    distance = math.hypot(dx, dy)
    gap = distance - me.radius - other.radius
    dot = dx * ux + dy * uy
    close = max(0., -dot / max(distance, 1e-9))
    current = clip01((cfg.gap_warn - gap) / (cfg.gap_warn - cfg.gap_critical))
    prediction = 0.
    u2 = ux * ux + uy * uy
    if dot < 0 and u2 > 1e-12:
        t = -dot / u2
        if 0 < t <= cfg.prediction_horizon:
            predicted_gap = math.hypot(dx + ux * t, dy + uy * t) - me.radius - other.radius
            prediction = clip01((cfg.predicted_gap_warn - predicted_gap) /
                                (cfg.predicted_gap_warn - cfg.gap_critical)) * (1 - t / cfg.prediction_horizon)
    return clip01(current + (1 - current) * prediction * clip01(close / cfg.closing_speed_ref))


def propagation_graph(people, cfg):
    edges = []
    for receiver in sorted(people, key=lambda p: p.id):
        raw = []
        for sender in sorted(people, key=lambda p: p.id):
            if sender.id == receiver.id:
                continue
            distance = math.hypot(sender.x - receiver.x, sender.y - receiver.y)
            weight = max(0., 1 - distance / cfg.spread_range) ** 2 * visibility(
                receiver, sender, cfg.spread_range, cfg.human_fov_deg, cfg.human_fov_fade_deg)
            if weight > 0:
                raw.append((sender.id, weight))
        denominator = max(1., sum(w for _, w in raw))
        edges.extend((sender, receiver.id, weight / denominator) for sender, weight in raw)
    return edges


def approach_conflict(me, intent, other, cfg):
    """Conservative feasibility of a temporary approach, using a visible neighbor only."""
    dx, dy = other.x-me.x, other.y-me.y
    ux, uy = other.vx-intent.desired_velocity_x, other.vy-intent.desired_velocity_y
    u2 = ux*ux+uy*uy
    t = max(0., min(cfg.prediction_horizon, -(dx*ux+dy*uy)/u2)) if u2 > 1e-12 else 0.
    predicted_gap = math.hypot(dx+ux*t, dy+uy*t)-me.radius-other.radius
    return predicted_gap < cfg.approach_clearance
