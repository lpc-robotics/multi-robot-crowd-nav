"""Novelty episodes use personal perception, with physical danger taking precedence."""
from dataclasses import replace
import math
from .behavior_v2 import IntentV3
from .perception import approach_conflict


def temporary(new, mode, target, stamp_ns, reason, **kwargs):
    previous = new.behavior
    new.behavior = replace(previous, mode=mode, target_robot_id=target, reason=reason,
        entered_ns=previous.entered_ns if previous.mode == mode else stamp_ns,
        target_since_ns=previous.target_since_ns if previous.target_robot_id == target else stamp_ns)
    return IntentV3(mode=mode, vigilant=previous.vigilant, has_target_robot=True, target_robot_id=target,
                    override_desired_velocity=True, allow_task_progress=False, **kwargs)


def surprise_step(me, old, new, base_intent, known, interest, visible, cfg, stamp_ns, dt):
    new.event = ''; new.exit_reason = ''; new.event_target_robot_id = 0
    new.surprise = old.surprise * math.exp(-dt / cfg.surprise_tau)
    new.visible_since_ns = {i: old.visible_since_ns.get(i, stamp_ns-round(dt*1e9)) for i in visible}
    danger = max((risk * confidence for _, risk, confidence in known.values()), default=0.)
    safe = danger <= cfg.surprise_max_risk and base_intent.mode != 'SCARED'
    target = old.behavior.target_robot_id
    if old.behavior.mode == 'SURPRISED':
        elapsed = (stamp_ns-old.behavior.entered_ns)/1e9
        reason = ('surprise_risk' if not safe else 'surprise_target_lost' if target not in visible else
                  'surprise_timeout' if elapsed >= cfg.surprise_timeout_s else '')
        if not reason:
            obj = known[target][0]
            yaw = math.atan2(obj.y-me.y, obj.x-me.x)
            aligned = (math.hypot(me.vx, me.vy) <= cfg.observation_stop_speed and
                       abs(math.remainder(yaw-me.yaw, 2*math.pi)) <= math.radians(cfg.observation_angle_deg))
            new.aligned_observation_s = old.aligned_observation_s+dt if aligned else 0.
            if new.surprise <= cfg.surprise_off and new.aligned_observation_s >= cfg.observation_hold_s+cfg.observation_display_margin_s:
                new.surprise_completed.append(target)
                reason = 'surprise_completed'
            else:
                new.behavior = replace(old.behavior)
                return temporary(new, 'SURPRISED', target, stamp_ns, 'observing', override_yaw=True,
                    desired_yaw=yaw, max_yaw_rate=cfg.observation_yaw_rate, yaw_stop_speed=cfg.observation_stop_speed)
        new.event = new.exit_reason = reason
        new.event_target_robot_id = target
        new.behavior.reason = reason
        new.aligned_observation_s = 0.
        return base_intent
    if safe and old.behavior.mode != 'THREATENING':
        candidates = [i for i in interest if i not in old.surprise_consumed and
            visible[i]['novelty'] >= cfg.surprise_novelty and
            (stamp_ns-new.visible_since_ns[i])/1e9 >= cfg.surprise_visible_s]
        if candidates:
            target = max(sorted(candidates), key=interest.get)
            new.surprise_consumed.append(target); new.surprise = 1.; new.aligned_observation_s = 0.
            new.event = 'surprise_started'
            new.event_target_robot_id = target
            obj = known[target][0]
            return temporary(new, 'SURPRISED', target, stamp_ns, new.event, override_yaw=True,
                desired_yaw=math.atan2(obj.y-me.y, obj.x-me.x), max_yaw_rate=cfg.observation_yaw_rate,
                yaw_stop_speed=cfg.observation_stop_speed)
    return base_intent


def segment_distance(point, start, end):
    dx, dy = end[0]-start[0], end[1]-start[1]
    length2 = dx*dx+dy*dy
    fraction = 0. if length2 == 0 else max(0., min(1., ((point[0]-start[0])*dx+(point[1]-start[1])*dy)/length2))
    return math.hypot(point[0]-start[0]-fraction*dx, point[1]-start[1]-fraction*dy)


def point_feasible(me, point, known, neighbors, cfg):
    if not (me.radius+cfg.approach_clearance <= point[0] <= cfg.world_width-me.radius-cfg.approach_clearance and
            me.radius+cfg.approach_clearance <= point[1] <= cfg.world_height-me.radius-cfg.approach_clearance):
        return False
    # Conservative straight-line reachability in the declared open world. No global plan.
    for obj in [*(r[0] for r in known.values()), *neighbors.values()]:
        if segment_distance((obj.x, obj.y), (me.x, me.y), point) < me.radius+obj.radius+cfg.approach_clearance:
            return False
    return True


def go_to(point, me, speed):
    dx, dy = point[0]-me.x, point[1]-me.y
    distance = math.hypot(dx, dy)
    magnitude = min(speed, distance)  # gain=1 s^-1; terminal approach without teleporting
    return (0., 0.) if distance == 0 else (magnitude*dx/distance, magnitude*dy/distance)


def front_point(me, robot, cfg):
    speed = math.hypot(robot.vx, robot.vy)
    heading = math.atan2(robot.vy, robot.vx)
    distance = me.radius+robot.radius+cfg.playful_front_gap+speed*cfg.playful_lookahead_s
    return robot.x+distance*math.cos(heading), robot.y+distance*math.sin(heading)


def playful_step(me, old, new, intent, known, visible, neighbors, profile, cfg, stamp_ns):
    """Conditional interaction state machine. All mutations are on the proposal."""
    if intent.mode == 'CURIOUS':
        target = intent.target_robot_id
        new.curious_target = target
        new.curious_since_ns = old.curious_since_ns if old.behavior.mode == 'CURIOUS' and old.curious_target == target else stamp_ns
    else:
        new.curious_target = 0; new.curious_since_ns = stamp_ns
    new.playful_blockers = []
    if profile.profile != 'playful':
        new.playful_blockers = ['ineligible_profile']; return intent

    def task_return(reason):
        new.phase = 'none'; new.target_point = None; new.playful_returning = True
        new.curious_target = 0; new.curious_since_ns = stamp_ns
        new.behavior = replace(new.behavior, mode='REGULAR', target_robot_id=0, entered_ns=stamp_ns,
                               target_since_ns=stamp_ns, reason=reason)
        return IntentV3(vigilant=new.behavior.vigilant)

    def finish(reason, completed=False):
        target = old.interaction_target or new.interaction_target
        new.cooldown_until_ns[target] = stamp_ns+round(cfg.playful_cooldown_s*1e9)
        new.event = 'playful_completed' if completed else 'playful_aborted'
        new.event_target_robot_id = target
        new.exit_reason = reason
        if not new.playful_outcome: new.playful_outcome = reason
        result = task_return(reason)
        # Emergency response retains precedence during the protected task return.
        if intent.mode == 'SCARED':
            new.behavior = replace(new.behavior, mode='SCARED', target_robot_id=intent.target_robot_id,
                                   reason=reason)
            return intent
        return result

    if old.playful_returning and old.phase == 'none':
        anchor = old.behavior.return_goal
        reached = not me.goals or anchor is None or me.goals[0] != anchor or math.dist((me.x, me.y), anchor) <= me.goal_radius
        if reached:
            new.playful_returning = False; new.event = 'playful_task_resumed'
            new.event_target_robot_id = old.interaction_target
        else:
            saved = new.behavior
            result = task_return('playful_task_return')
            if intent.mode == 'SCARED': new.behavior = saved; return intent
            return result

    danger = max((r*confidence for _,r,confidence in known.values()), default=0.)
    if old.phase != 'none':
        target = old.interaction_target
        obj = known[target][0] if target in known else None
        if intent.mode == 'SCARED' or danger >= cfg.emergency_risk:
            return finish('playful_emergency')
        # Entry requires visibility; a short sidestep may turn the body away.
        # Continue only while existing bounded personal memory remains valid.
        if obj is None:
            return finish('playful_target_lost')
        elapsed = (stamp_ns-old.phase_since_ns)/1e9
        if old.phase == 'yield':
            if math.dist((me.x, me.y), old.target_point) <= cfg.playful_arrival_radius:
                return finish('yield_reached', completed=True)
            if elapsed >= cfg.playful_yield_s:
                return finish('yield_timeout')
            vx, vy = go_to(old.target_point, me, cfg.playful_speed)
            new.behavior = replace(old.behavior)
            return temporary(new, 'THREATENING', target, stamp_ns, 'yield', desired_velocity_x=vx, desired_velocity_y=vy)

        speed = math.hypot(obj.vx, obj.vy)
        heading = math.atan2(obj.vy, obj.vx)
        turn = abs(math.remainder(heading-old.robot_heading, 2*math.pi))
        reason = ('robot_stopped' if speed < cfg.playful_min_robot_speed else
                  'robot_turned' if turn > math.radians(cfg.playful_turn_limit_deg) else
                  'playful_risk' if danger > cfg.playful_max_risk or new.variables['fear'] > cfg.playful_max_fear else '')
        if not reason and old.phase == 'intercept':
            # Intercept the moving robot's personally observed forward region.
            # Freeze this point on occupancy; never reset the episode clock.
            new.target_point = front_point(me, obj, cfg)
            if not point_feasible(me, new.target_point, known, neighbors, cfg): reason = 'intercept_unreachable'
            elif math.dist((me.x, me.y), new.target_point) <= cfg.playful_arrival_radius:
                new.phase = 'occupy'; new.phase_since_ns = stamp_ns; new.event = 'playful_occupied'
                new.event_target_robot_id = target
            elif elapsed >= cfg.playful_intercept_s: reason = 'intercept_timeout'
        if not reason and old.phase == 'occupy' and elapsed >= cfg.playful_occupy_s:
            reason = 'occupancy_complete'
        if not reason:
            vx, vy = go_to(new.target_point, me, cfg.playful_speed) if new.phase == 'intercept' else (0., 0.)
            candidate = temporary(new, 'THREATENING', target, stamp_ns, new.phase,
                desired_velocity_x=vx, desired_velocity_y=vy, override_yaw=new.phase == 'occupy',
                desired_yaw=math.atan2(obj.y-me.y, obj.x-me.x), max_yaw_rate=cfg.observation_yaw_rate,
                yaw_stop_speed=cfg.observation_stop_speed)
            # Preserve episode entry time; the base selector only knows legacy modes.
            new.behavior.entered_ns = old.behavior.entered_ns
            if not any(approach_conflict(me, candidate, n, cfg) for n in neighbors.values()):
                return candidate
            reason = 'playful_neighbor_conflict'
        new.playful_outcome = reason
        # No chasing the robot after it stops. Pick a feasible lateral yield point once.
        nx, ny = -math.sin(old.robot_heading), math.cos(old.robot_heading)
        choices = [(me.x+sign*nx*cfg.playful_yield_distance, me.y+sign*ny*cfg.playful_yield_distance) for sign in (-1, 1)]
        choices = [p for p in choices if point_feasible(me, p, known, neighbors, cfg)]
        if not choices: return finish('yield_unreachable')
        anchor = old.behavior.return_goal or (me.goals[0] if me.goals else (me.x, me.y))
        new.target_point = min(choices, key=lambda p: (math.dist(p, anchor), p))
        new.phase = 'yield'; new.phase_since_ns = stamp_ns; new.event = 'playful_yield_started'; new.exit_reason = reason
        new.event_target_robot_id = target
        vx, vy = go_to(new.target_point, me, cfg.playful_speed)
        new.behavior = replace(old.behavior)
        return temporary(new, 'THREATENING', target, stamp_ns, 'yield', desired_velocity_x=vx, desired_velocity_y=vy)

    target = intent.target_robot_id
    checks = {
        'disabled': cfg.playful_enabled == 0,
        'not_curious': intent.mode != 'CURIOUS',
        'observation_incomplete': target not in new.surprise_completed,
        'curious_duration': (stamp_ns-new.curious_since_ns)/1e9 < cfg.playful_curious_s,
        'activation': new.variables['curiosity'] < cfg.playful_curiosity,
        'fear': new.variables['fear'] > cfg.playful_max_fear,
        'risk': danger > cfg.playful_max_risk,
        'not_visible': target not in visible,
        'preference': bool(profile.preferred_robot_id and target != profile.preferred_robot_id),
        'cooldown': stamp_ns < new.cooldown_until_ns.get(target, 0),
        'budget': new.episode_counts.get(target, 0) >= cfg.playful_max_episodes,
    }
    new.playful_blockers = [k for k,v in checks.items() if v]
    if new.playful_blockers: return intent
    obj = known[target][0]; speed = math.hypot(obj.vx, obj.vy)
    if speed < cfg.playful_min_robot_speed:
        new.playful_blockers = ['robot_stationary']; return intent
    heading = math.atan2(obj.vy, obj.vx)
    point = front_point(me, obj, cfg)
    if math.dist((me.x, me.y), point) > cfg.playful_speed*cfg.playful_intercept_s or not point_feasible(me, point, known, neighbors, cfg):
        new.playful_blockers = ['front_point_unreachable']; return intent
    vx, vy = go_to(point, me, cfg.playful_speed)
    candidate = IntentV3(mode='THREATENING', has_target_robot=True, target_robot_id=target,
        override_desired_velocity=True, allow_task_progress=False, desired_velocity_x=vx, desired_velocity_y=vy)
    if any(approach_conflict(me, candidate, n, cfg) for n in neighbors.values()):
        new.playful_blockers = ['front_path_neighbor_conflict']; return intent
    new.phase = 'intercept'; new.phase_since_ns = stamp_ns; new.interaction_target = target
    new.target_point = point; new.robot_heading = heading; new.playful_outcome = ''
    new.episode_counts[target] = new.episode_counts.get(target, 0)+1; new.event = 'playful_started'
    new.event_target_robot_id = target
    return temporary(new, 'THREATENING', target, stamp_ns, 'intercept', desired_velocity_x=vx, desired_velocity_y=vy)
