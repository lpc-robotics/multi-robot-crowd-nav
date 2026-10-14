"""Robot-stimulated dual SIS activation, synchronous and simulation-time based."""
import copy
from dataclasses import dataclass, field, asdict, replace
import math
from .psychology_config import PsychologyConfig
from .perception import visibility, risk, propagation_graph, approach_conflict
from .behavior import BehaviorState, RobotMemory, NeighborMemory, choose_behavior, avoid_neighbors, Intent, MODES
from .psychology import Entity, MentalState, Modifiers, Proposal


@dataclass
class CrowdMentalState(MentalState):
    variables: dict = field(default_factory=lambda: {"curiosity": 0., "fear": 0.})
    behavior: BehaviorState = field(default_factory=BehaviorState)
    memories: dict[int, RobotMemory] = field(default_factory=dict)
    neighbors: dict[int, NeighborMemory] = field(default_factory=dict)

    def validate(self):
        if set(self.variables) != {"curiosity", "fear"} or not all(0 <= v <= 1 for v in self.variables.values()):
            raise ValueError("activation outside [0,1]")
        b = self.behavior
        if b.mode not in MODES or min(b.entered_ns,b.target_since_ns,b.approach_blocked_until_ns)<0:
            raise ValueError("invalid behavior state")
        if b.target_robot_id and b.target_robot_id not in self.memories:
            raise ValueError("behavior target has no observation memory")
        for key, m in self.memories.items():
            if key != m.id or key >= 0 or m.last_seen_ns < 0 or not 0 <= m.novelty <= 1:
                raise ValueError("invalid robot memory")
            if not all(math.isfinite(v) for v in (m.x, m.y, m.vx, m.vy, m.radius)) or m.radius <= 0:
                raise ValueError("invalid memory geometry")
        if type(b.returning_to_task) is not bool:
            raise ValueError("invalid task return state")
        if b.return_goal is not None and (len(b.return_goal) != 2 or not all(math.isfinite(v) for v in b.return_goal)):
            raise ValueError("invalid task return goal")
        for key, m in self.neighbors.items():
            if key != m.id or key <= 0 or m.last_seen_ns < 0 or m.radius <= 0 or not all(
                    math.isfinite(v) for v in (m.x,m.y,m.vx,m.vy,m.radius)):
                raise ValueError("invalid neighbor memory")


def exponential_step(value, activation, recovery, dt):
    rate = activation + recovery
    if not all(math.isfinite(v) for v in (value, activation, recovery, dt, rate)) or min(activation, recovery, dt) < 0:
        raise ValueError("invalid SIS update")
    result = value if rate == 0 else value + (activation / rate - value) * (-math.expm1(-rate * dt))
    if not -1e-12 <= result <= 1 + 1e-12:
        raise ValueError("SIS update outside [0,1]")
    return min(1., max(0., result))


class DualSISPsychology:
    name = "sis_cf_v1"
    version = "1.0"

    def initialize(self, config, agent_ids, seed):
        self.config = PsychologyConfig.parse(config)
        self.agent_ids = tuple(sorted(agent_ids)); self.seed = seed

    reset = initialize

    def initial_states(self):
        return {i: CrowdMentalState() for i in self.agent_ids}

    def snapshot(self):
        return {"model": self.name, "version": self.version, "schema_version": 2,
                "config_hash": self.config.digest, "agent_ids": list(self.agent_ids), "seed": self.seed}

    def restore(self, snapshot):
        if snapshot != self.snapshot():
            raise ValueError("incompatible psychology snapshot")

    def step(self, context, previous_state):
        cfg = self.config
        if set(previous_state) != set(self.agent_ids) or {p.id for p in context.people} != set(self.agent_ids):
            raise ValueError("psychology identities changed")
        if context.stamp_ns < 0 or not math.isfinite(context.dt) or not 0 < context.dt <= .025000001:
            raise ValueError("invalid simulation time step")
        if len({r.id for r in context.robots}) != len(context.robots) or any(r.id >= 0 for r in context.robots):
            raise ValueError("invalid robot identities")
        for obj in (*context.people, *context.robots):
            if not all(math.isfinite(v) for v in (obj.x, obj.y, obj.vx, obj.vy, obj.yaw, obj.radius)) or obj.radius <= 0:
                raise ValueError("invalid perception input")
        for state in previous_state.values():
            state.validate()
        end_ns = context.stamp_ns + round(context.dt * 1e9)
        cues = {i: (cfg.expressivity_C * s.variables["curiosity"], cfg.expressivity_F * s.variables["fear"])
                for i, s in previous_state.items()}
        edges = propagation_graph(context.people, cfg)
        social = {i: [0., 0.] for i in self.agent_ids}
        edge_log = []
        for sender, receiver, weight in edges:
            yc, yf = cues[sender]
            social[receiver][0] += weight * yc; social[receiver][1] += weight * yf
            edge_log.append({"source_id": sender, "target_id": receiver, "weight": weight, "cue_C": yc, "cue_F": yf})
        proposed = {}; intents = {}; diagnostics = {}
        for me in sorted(context.people, key=lambda p: p.id):
            old = previous_state[me.id]; new = copy.deepcopy(old)
            known = {}; interest = {}; visible = {}; fear_complement = curiosity_complement = 1.
            for robot in sorted(context.robots, key=lambda r: r.id):
                v = visibility(me, robot, cfg.robot_visual_range, cfg.robot_fov_deg, cfg.robot_fov_fade_deg)
                if v <= 0:
                    continue
                novelty = old.memories[robot.id].novelty if robot.id in old.memories else 1.
                r = risk(me, robot, cfg)
                fear_complement *= 1 - v * r; curiosity_complement *= 1 - v * novelty
                interest[robot.id] = v * novelty * (1 - r)
                known[robot.id] = (robot, r * v, 1.)
                visible[robot.id] = {"visibility": v, "risk": r, "novelty": novelty, "knowledge": "visible"}
                new.memories[robot.id] = RobotMemory(robot.id, robot.name, robot.x, robot.y, robot.vx, robot.vy,
                    robot.radius, context.stamp_ns, novelty * math.exp(-cfg.novelty_decay * v * context.dt))
            for i, memory in old.memories.items():
                if i in known:
                    continue
                age = (context.stamp_ns - memory.last_seen_ns) / 1e9
                if age < 0:
                    raise ValueError("memory from future")
                confidence = math.exp(-age / cfg.memory_tau)
                if age <= cfg.memory_ttl and confidence >= cfg.minimum_memory_confidence:
                    horizon = min(age, cfg.memory_prediction_horizon)
                    predicted = Entity(i, memory.name, memory.x + memory.vx * horizon, memory.y + memory.vy * horizon,
                                       memory.vx, memory.vy, memory.radius)
                    known[i] = (predicted, risk(me, predicted, cfg), confidence)
            sf = 1 - fear_complement; sc = (1 - sf) * (1 - curiosity_complement)
            lc = cfg.alpha_C * sc + cfg.beta_C * social[me.id][0]
            lf = cfg.alpha_F * sf + cfg.beta_F * social[me.id][1]
            c0, f0 = old.variables["curiosity"], old.variables["fear"]
            c = exponential_step(c0, lc, cfg.mu_C + cfg.kappa * f0, context.dt)
            f = exponential_step(f0, lf, cfg.mu_F, context.dt)
            new.variables = {"curiosity": c, "fear": f}
            new.behavior, intent = choose_behavior(me, c, f, old.behavior, known, interest, sf, end_ns, cfg)
            # Temporary behaviors can leave an agent far from its original route.
            # Keep anticipating neighbors until it reaches the saved next goal.
            anchor = old.behavior.return_goal or (me.goals[0] if me.goals else None)
            reached = not me.goals or (anchor is not None and (me.goals[0] != anchor or
                math.hypot(me.x-anchor[0],me.y-anchor[1]) <= me.goal_radius))
            new.behavior.returning_to_task = intent.mode != "REGULAR" or (old.behavior.returning_to_task and not reached)
            new.behavior.return_goal = (anchor if old.behavior.returning_to_task and not reached else
                (me.goals[0] if me.goals else None)) if new.behavior.returning_to_task else None
            neighbors = {}
            new.neighbors = {}
            for p in sorted(context.people,key=lambda p:p.id):
                if p.id == me.id:
                    continue
                if visibility(me,p,cfg.human_visual_range,cfg.human_fov_deg,cfg.human_fov_fade_deg)>0:
                    new.neighbors[p.id] = NeighborMemory(p.id,p.name,p.x,p.y,p.vx,p.vy,p.radius,context.stamp_ns)
                    neighbors[p.id] = p
                elif p.id in old.neighbors:
                    memory = old.neighbors[p.id]
                    age = (context.stamp_ns-memory.last_seen_ns)/1e9
                    if age < 0:
                        raise ValueError("neighbor memory from future")
                    if age <= cfg.memory_ttl and math.exp(-age/cfg.memory_tau) >= cfg.minimum_memory_confidence:
                        new.neighbors[p.id] = memory
                        horizon = min(age,cfg.memory_prediction_horizon)
                        neighbors[p.id] = Entity(p.id,memory.name,memory.x+memory.vx*horizon,
                            memory.y+memory.vy*horizon,memory.vx,memory.vy,memory.radius)
            conflicts = []
            if intent.mode == "CURIOUS":
                conflicts = [p.id for p in neighbors.values() if
                    approach_conflict(me,intent,p,cfg)]
                if conflicts:
                    new.behavior = replace(new.behavior, mode="REGULAR", target_robot_id=0, entered_ns=end_ns,
                        target_since_ns=end_ns, reason="approach_blocked",
                        approach_blocked_until_ns=end_ns+round(cfg.approach_retry_delay*1e9))
                    intent = Intent(vigilant=new.behavior.vigilant, override_desired_velocity=True)
            avoidance_conflicts = []; avoidance_feasible = True
            if new.behavior.returning_to_task or intent.override_desired_velocity:
                intent, avoidance_conflicts, avoidance_feasible = avoid_neighbors(me,intent,neighbors.values(),cfg)
            proposed[me.id] = new; intents[me.id] = intent
            target = new.behavior.target_robot_id
            diagnostics[me.id] = {"old": dict(old.variables), "new": dict(new.variables), "direct_C": sc, "direct_F": sf,
                "social_C": social[me.id][0], "social_F": social[me.id][1], "lambda_C": lc, "lambda_F": lf,
                "recovery_C": cfg.mu_C * c0, "recovery_F": cfg.mu_F * f0, "inhibition_C": cfg.kappa * f0 * c0,
                "derivative_semantics": "pre_step_instantaneous", "behavior": asdict(new.behavior),
                "visible_robots": visible, "target_age_s": None if not target else
                (context.stamp_ns - new.memories[target].last_seen_ns) / 1e9,
                "target_knowledge": "none" if not target else ("visible" if target in visible else "memory"),
                "intent": asdict(intent)}
            diagnostics[me.id]["approach_conflicts"] = sorted(conflicts)
            diagnostics[me.id]["avoidance_conflicts"] = sorted(avoidance_conflicts)
            diagnostics[me.id]["avoidance_feasible"] = avoidance_feasible
            diagnostics[me.id]["target_confidence"] = None if not target else known[target][2]
        return Proposal(proposed, {i: Modifiers() for i in self.agent_ids}, intents,
                        {"agents": diagnostics, "edges": edge_log, "event_stamp_ns": end_ns})
