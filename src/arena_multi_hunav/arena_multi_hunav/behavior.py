"""Behavior intent uses only current observations and valid personal memory."""
from dataclasses import dataclass, field
import math

MODES = {"REGULAR": 0, "CURIOUS": 1, "SCARED": 2}


@dataclass(frozen=True)
class Intent:
    mode: str = "REGULAR"
    vigilant: bool = False
    has_target_robot: bool = False
    target_robot_id: int = 0
    override_desired_velocity: bool = False
    desired_velocity_x: float = 0.
    desired_velocity_y: float = 0.
    allow_task_progress: bool = True

    def validate(self):
        if self.mode not in MODES or not all(math.isfinite(v) for v in (self.desired_velocity_x, self.desired_velocity_y)):
            raise ValueError("invalid behavior intent")
        if math.hypot(self.desired_velocity_x, self.desired_velocity_y) > 1 + 1e-9:
            raise ValueError("intent exceeds hard speed cap")
        if self.has_target_robot != (self.target_robot_id < 0):
            raise ValueError("invalid intent target")


@dataclass
class BehaviorState:
    mode: str = "REGULAR"
    vigilant: bool = False
    entered_ns: int = 0
    target_robot_id: int = 0
    target_since_ns: int = 0
    reason: str = "initial"
    approach_blocked_until_ns: int = 0
    returning_to_task: bool = False
    return_goal: tuple | None = None


@dataclass
class NeighborMemory:
    id: int
    name: str
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    last_seen_ns: int


@dataclass
class RobotMemory:
    id: int
    name: str
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    last_seen_ns: int
    novelty: float = 1.


def select_target(scores, previous, stamp_ns, cfg, emergency=False):
    if not scores:
        return 0
    best = max(sorted(scores), key=scores.get)
    if previous.target_robot_id in scores and not emergency:
        old = previous.target_robot_id
        if (stamp_ns - previous.target_since_ns) / 1e9 < cfg.target_min_hold or scores[best] <= scores[old] + cfg.target_switch_margin:
            return old
    return best


def choose_behavior(me, c, f, previous, known, visible_interest, direct_fear, stamp_ns, cfg):
    # known maps robot id to (personally observed/predicted entity, risk, confidence).
    threats = {i: r * confidence for i, (_, r, confidence) in known.items() if r > 0}
    emergency = max(threats.values(), default=0.) >= cfg.emergency_risk
    vigilance = (f > cfg.vigilance_off if previous.vigilant else f >= cfg.vigilance_on)
    curious_safe = f <= cfg.curiosity_max_fear and direct_fear <= cfg.curiosity_max_risk
    target = 0; mode = "REGULAR"; reason = "task"
    held = (stamp_ns - previous.entered_ns) / 1e9 < cfg.min_mode_hold
    if emergency or (threats and (f >= cfg.fear_on or
            (previous.mode == "SCARED" and (f > cfg.fear_off or held)))):
        mode = "SCARED"; reason = "emergency_override" if emergency else "fear"
        target = select_target(threats, previous, stamp_ns, cfg, emergency)
    elif curious_safe and stamp_ns >= previous.approach_blocked_until_ns:
        interest = dict(visible_interest)
        if previous.mode == "CURIOUS" and previous.target_robot_id in known:
            i = previous.target_robot_id
            obj, r, confidence = known[i]
            if r <= cfg.curiosity_max_risk:
                interest.setdefault(i, confidence * (1 - r))
        if interest and (c >= cfg.curiosity_on or
                (previous.mode == "CURIOUS" and (c > cfg.curiosity_off or held))):
            mode = "CURIOUS"; reason = "curiosity"
            target = select_target(interest, previous, stamp_ns, cfg)
    if mode == "REGULAR":
        if f >= cfg.fear_on and not threats:
            reason = "unknown_threat"
        elif not curious_safe and previous.mode == "CURIOUS":
            reason = "approach_interrupted"
        elif previous.mode != "REGULAR":
            reason = "target_or_activation_expired"
    state = BehaviorState(mode, vigilance, previous.entered_ns if mode == previous.mode else stamp_ns,
                          target, previous.target_since_ns if target == previous.target_robot_id else stamp_ns, reason,
                          previous.approach_blocked_until_ns)
    vx = vy = 0.; override = False
    if mode == "REGULAR" and stamp_ns < previous.approach_blocked_until_ns:
        override = True; state.reason = "approach_wait"
    elif mode == "REGULAR" and vigilance:
        override = True
        if me.goals:
            dx, dy = me.goals[0][0] - me.x, me.goals[0][1] - me.y
            distance = math.hypot(dx, dy)
            if distance > me.goal_radius:
                speed = min(1., me.desired_velocity) * (1 - cfg.vigilant_speed_gain * f)
                vx, vy = speed * dx / distance, speed * dy / distance
    elif mode != "REGULAR":
        override = True
        obj = known[target][0]
        dx, dy = obj.x - me.x, obj.y - me.y
        distance = math.hypot(dx, dy)
        if distance <= 1e-9:
            dx, dy, distance = 1., 0., 1.
            gap = -me.radius - obj.radius
        else:
            gap = distance - me.radius - obj.radius
        if mode == "CURIOUS":
            speed = max(-cfg.curiosity_retreat_cap, min(cfg.curiosity_speed, cfg.curiosity_gap_gain * (gap - cfg.curiosity_standoff_gap)))
            vx, vy = speed * dx / distance, speed * dy / distance
        else:
            # All personally known threats contribute; cancellation falls back to the selected threat.
            ax = ay = 0.; speed = 0.
            for item, r, confidence in known.values():
                tx, ty = me.x - item.x, me.y - item.y
                norm = math.hypot(tx, ty)
                if r > 0:
                    if norm > 1e-9:
                        ax += r * confidence * tx / norm; ay += r * confidence * ty / norm
                    speed = max(speed, min(cfg.scared_speed, max(0., cfg.scared_gap_gain *
                        (cfg.scared_target_gap - (norm - me.radius - item.radius)))))
            norm = math.hypot(ax, ay)
            if norm <= 1e-9:
                ax, ay, norm = -dx, -dy, distance
            vx, vy = speed * ax / norm, speed * ay / norm
    intent = Intent(mode, vigilance, bool(target), target, override, vx, vy, mode == "REGULAR")
    intent.validate()
    return state, intent


def avoid_neighbors(me, intent, neighbors, cfg):
    """Closest feasible desired velocity within local separation half planes.

    This is behavioral anticipation, not a hard contact solver. Neighbors must
    already be restricted to observations or bounded personal memory. Physical
    forces and integration remain in the core, so geometry still needs testing.
    """
    from dataclasses import replace
    if intent.override_desired_velocity:
        preferred = (intent.desired_velocity_x, intent.desired_velocity_y)
    else:
        preferred = (0., 0.)
        if me.goals:
            dx, dy = me.goals[0][0]-me.x, me.goals[0][1]-me.y
            distance = math.hypot(dx, dy)
            if distance > me.goal_radius:
                speed = min(1., me.desired_velocity)
                preferred = (speed*dx/distance, speed*dy/distance)
    planes = []
    for other in sorted(neighbors, key=lambda p: p.id):
        dx, dy = me.x-other.x, me.y-other.y
        distance = math.hypot(dx, dy)
        if distance < 1e-9:
            nx, ny = (1. if me.id < other.id else -1.), 0.
        else:
            nx, ny = dx/distance, dy/distance
        gap = distance-me.radius-other.radius
        bound = nx*other.vx+ny*other.vy-(gap-cfg.approach_clearance)/cfg.prediction_horizon
        planes.append((nx, ny, bound, other.id))
    def feasible(v):
        return math.hypot(*v) <= 1.+1e-9 and all(nx*v[0]+ny*v[1] >= b-1e-9 for nx,ny,b,_ in planes)
    conflicts = [i for nx,ny,b,i in planes if nx*preferred[0]+ny*preferred[1] < b]
    if not conflicts:
        return intent, [], True
    # A convex disk/half-plane intersection has its nearest point on an edge,
    # an edge intersection, or an edge/circle intersection. Enumerate in 2-D.
    candidates = [(0., 0.)]
    for k,(nx,ny,b,_) in enumerate(planes):
        delta = b-nx*preferred[0]-ny*preferred[1]
        candidates.append((preferred[0]+delta*nx, preferred[1]+delta*ny))
        if abs(b)<=1:
            side = math.sqrt(max(0.,1-b*b))
            candidates.extend(((b*nx-side*ny,b*ny+side*nx),(b*nx+side*ny,b*ny-side*nx)))
        for mx,my,c,_ in planes[:k]:
            det=nx*my-ny*mx
            if abs(det)>1e-10:
                candidates.append(((b*my-ny*c)/det,(nx*c-b*mx)/det))
    valid=[v for v in candidates if feasible(v)]
    if valid:
        velocity=min(valid,key=lambda v:((v[0]-preferred[0])**2+(v[1]-preferred[1])**2,v))
    else:
        # Infeasible congestion is observable; choose the least violating bounded
        # candidate. Do not teleport agents or claim collision-free guarantees.
        bounded=[(x/max(1.,math.hypot(x,y)),y/max(1.,math.hypot(x,y))) for x,y in candidates]
        velocity=min(bounded,key=lambda v:(max(max(0.,b-nx*v[0]-ny*v[1]) for nx,ny,b,_ in planes),
            (v[0]-preferred[0])**2+(v[1]-preferred[1])**2,v))
    return replace(intent,override_desired_velocity=True,desired_velocity_x=velocity[0],desired_velocity_y=velocity[1]), conflicts, bool(valid)
