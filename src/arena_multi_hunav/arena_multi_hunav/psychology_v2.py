"""Robot-stimulated dual SIS activation, synchronous and simulation-time based."""
import copy
from dataclasses import dataclass, field, asdict, replace
import math
from .psychology_v2_config import PsychologyConfigV2
from .psychology_sis import exponential_step
from .psychology_v2_state import CrowdMentalStateV2
from .interaction_v2 import surprise_step, playful_step
from .behavior_v2 import IntentV3
from .perception import visibility, risk, propagation_graph, approach_conflict
from .behavior import BehaviorState, RobotMemory, NeighborMemory, choose_behavior, avoid_neighbors, Intent, MODES
from .psychology import Entity, MentalState, Modifiers, Proposal


class SocialPsychologyV2:
    name = "sis_cf_v2"
    version = "2.0"

    def initialize(self, config, agent_ids, seed):
        self.config = PsychologyConfigV2.parse(config)
        if set(self.config.individuals) - set(agent_ids):
            raise ValueError("individual config contains unknown pedestrian ids")
        self.resolved = {i: self.config.resolved(i) for i in agent_ids}
        self.agent_ids = tuple(sorted(agent_ids)); self.seed = seed
        # Configuration is immutable for an epoch. Full resolved parameters are
        # frozen once in source_manifest.json, not serialised four times per tick.
        self.config_hash = self.config.digest

    reset = initialize

    def initial_states(self):
        return {i: CrowdMentalStateV2() for i in self.agent_ids}

    def snapshot(self):
        return {"model": self.name, "version": self.version, "schema_version": 3,
                "config_hash": self.config_hash, "agent_ids": list(self.agent_ids), "seed": self.seed}

    def restore(self, snapshot):
        if snapshot != self.snapshot():
            raise ValueError("incompatible psychology snapshot")

    def step(self, context, previous_state):
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
            if state.last_stamp_ns and context.stamp_ns != state.last_stamp_ns:
                raise ValueError("V2 requires contiguous committed simulation time")
        end_ns = context.stamp_ns + round(context.dt * 1e9)
        active = [p for p in context.people if self.resolved[p.id][0].profile not in ("immune_dynamic", "impassive_static")]
        cues = {p.id: (self.resolved[p.id][1].expressivity_C * previous_state[p.id].variables["curiosity"],
                     self.resolved[p.id][1].expressivity_F * previous_state[p.id].variables["fear"]) for p in active}
        graph_cache = {}; edges = []
        for p in active:
            cfg = self.resolved[p.id][1]
            key = (cfg.spread_range, cfg.human_fov_deg, cfg.human_fov_fade_deg)
            if key not in graph_cache:
                graph_cache[key] = propagation_graph(active, cfg)
            edges.extend(e for e in graph_cache[key] if e[1] == p.id)
        social = {i: [0., 0.] for i in self.agent_ids}
        edge_log = []
        for sender, receiver, weight in edges:
            yc, yf = cues[sender]
            social[receiver][0] += weight * yc; social[receiver][1] += weight * yf
            edge_log.append({"source_id": sender, "target_id": receiver, "weight": weight, "cue_C": yc, "cue_F": yf})
        proposed = {}; intents = {}; diagnostics = {}
        for me in sorted(context.people, key=lambda p: p.id):
            profile, cfg = self.resolved[me.id]
            old = previous_state[me.id]; new = copy.deepcopy(old)
            new.last_stamp_ns = end_ns
            if profile.profile in ("immune_dynamic", "impassive_static"):
                new = CrowdMentalStateV2(last_stamp_ns=end_ns)
                intent = IntentV3(robot_response=int(profile.profile == "impassive_static"))
                proposed[me.id] = new; intents[me.id] = intent
                diagnostics[me.id] = {"profile": profile.profile, "old": dict(old.variables), "new": dict(new.variables),
                    "direct_C": 0., "direct_F": 0., "social_C": 0., "social_F": 0.,
                    "behavior": asdict(new.behavior), "intent": asdict(intent), "target_knowledge": "none"}
                continue
            known = {}; interest = {}; visible = {}; fear_complement = curiosity_complement = 1.
            for robot in sorted(context.robots, key=lambda r: r.id):
                v = visibility(me, robot, cfg.robot_visual_range, cfg.robot_fov_deg, cfg.robot_fov_fade_deg)
                if v <= 0:
                    continue
                novelty = old.memories[robot.id].novelty if robot.id in old.memories else 1.
                r = risk(me, robot, cfg)
                fear_complement *= 1 - v * r; curiosity_complement *= 1 - v * novelty
                if not profile.preferred_robot_id or robot.id == profile.preferred_robot_id:
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
            intent = surprise_step(me, old, new, IntentV3.from_legacy(intent), known, interest, visible, cfg, end_ns, context.dt)
            conflicts = []
            if intent.mode == "CURIOUS":
                conflicts = [p.id for p in neighbors.values() if
                    approach_conflict(me,intent,p,cfg)]
                if conflicts:
                    new.behavior = replace(new.behavior, mode="REGULAR", target_robot_id=0, entered_ns=end_ns,
                        target_since_ns=end_ns, reason="approach_blocked",
                        approach_blocked_until_ns=end_ns+round(cfg.approach_retry_delay*1e9))
                    intent = IntentV3(vigilant=new.behavior.vigilant, override_desired_velocity=True)
            intent = playful_step(me, old, new, intent, known, visible, neighbors, profile, cfg, end_ns)
            # Temporary behaviors can leave an agent far from its original route.
            # Keep anticipating neighbors until it reaches the saved next goal.
            anchor = old.behavior.return_goal or (me.goals[0] if me.goals else None)
            reached = not me.goals or (anchor is not None and (me.goals[0] != anchor or
                math.hypot(me.x-anchor[0],me.y-anchor[1]) <= me.goal_radius))
            new.behavior.returning_to_task = intent.mode != "REGULAR" or (old.behavior.returning_to_task and not reached)
            new.behavior.return_goal = (anchor if old.behavior.returning_to_task and not reached else
                (me.goals[0] if me.goals else None)) if new.behavior.returning_to_task else None
            avoidance_conflicts = []; avoidance_feasible = True
            if intent.mode != "SURPRISED" and (new.behavior.returning_to_task or intent.override_desired_velocity):
                intent, avoidance_conflicts, avoidance_feasible = avoid_neighbors(me,intent,neighbors.values(),cfg)
            proposed[me.id] = new; intents[me.id] = intent
            target = new.behavior.target_robot_id
            diagnostics[me.id] = {"profile": profile.profile, "old": dict(old.variables), "new": dict(new.variables), "direct_C": sc, "direct_F": sf,
                "social_C": social[me.id][0], "social_F": social[me.id][1], "lambda_C": lc, "lambda_F": lf,
                "recovery_C": cfg.mu_C * c0, "recovery_F": cfg.mu_F * f0, "inhibition_C": cfg.kappa * f0 * c0,
                "derivative_semantics": "pre_step_instantaneous", "behavior": asdict(new.behavior),
                "surprise": new.surprise, "event": new.event, "exit_reason": new.exit_reason,
                "phase": new.phase, "motivation": "playful" if new.phase != "none" else "none",
                "playful_blockers": new.playful_blockers, "playful_outcome": new.playful_outcome,
                "episode_counts": new.episode_counts, "cooldown_until_ns": new.cooldown_until_ns,
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
