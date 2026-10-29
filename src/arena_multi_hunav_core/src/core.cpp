#include "arena_multi_hunav_core/core.hpp"
#include "sfm.hpp"
#include <algorithm>
#include <set>
#include <map>
#include <type_traits>
#include <stdexcept>
#include <rclcpp/time.hpp>

namespace arena_multi_hunav_core {
namespace {
using Agent = hunav_msgs::msg::Agent;
using Modifier = arena_multi_hunav_msgs::msg::BehaviorModifiers;
void require(bool valid, const char * reason) { if (!valid) throw std::invalid_argument(reason); }
void finite(double value) { require(std::isfinite(value), "non-finite input"); }
sfm::Agent convert(const Agent & a) {
  for (double v : {a.position.position.x, a.position.position.y, double(a.yaw),
      a.velocity.linear.x, a.velocity.linear.y, double(a.radius), double(a.desired_velocity)}) finite(v);
  require(a.radius > 0 && a.desired_velocity >= 0, "invalid radius or speed");
  sfm::Agent s;
  s.id = a.id; s.groupId = -1; s.radius = a.radius; s.desiredVelocity = a.desired_velocity;
  s.position.set(a.position.position.x, a.position.position.y);
  s.velocity.set(a.velocity.linear.x, a.velocity.linear.y); s.yaw.setRadian(a.yaw);
  s.cyclicGoals = a.cyclic_goals;
  require(a.type == Agent::ROBOT || a.goal_radius > 0, "invalid goal radius");
  for (const auto & g : a.goals) {
    finite(g.position.x); finite(g.position.y);
    sfm::Goal goal; goal.center.set(g.position.x, g.position.y); goal.radius = a.goal_radius;
    s.goals.push_back(goal);
  }
  for (double v : {double(a.behavior.goal_force_factor), double(a.behavior.obstacle_force_factor), double(a.behavior.social_force_factor)}) {
    finite(v); require(v >= 0, "negative force coefficient");
  }
  s.params.forceFactorDesired = a.behavior.goal_force_factor;
  s.params.forceFactorObstacle = a.behavior.obstacle_force_factor;
  s.params.forceFactorSocial = a.behavior.social_force_factor;
  return s;
}
// Deterministic pair direction also covers exact overlaps without division by zero.
utils::Vector2d direction(const sfm::Agent & a, const sfm::Agent & b) {
  auto d = a.position - b.position;
  if (d.norm() < 1e-9) return utils::Vector2d(a.id < b.id ? -1.0 : 1.0, 0.0);
  return d / d.norm();
}
double robot_visibility(const sfm::Agent & me, const sfm::Agent & robot, const Config & cfg) {
  const auto diff = robot.position - me.position;
  // A coincident center has no bearing. Near-repulsion handles its geometry.
  if (cfg.robot_fov_deg == 360.0 || (diff.getX() == 0 && diff.getY() == 0)) return 1.0;
  const double forward = diff.getX() * me.yaw.cos() + diff.getY() * me.yaw.sin();
  const double lateral = -diff.getX() * me.yaw.sin() + diff.getY() * me.yaw.cos();
  const double angle_deg = std::abs(std::atan2(lateral, forward)) * 180.0 / std::acos(-1.0);
  const double half = cfg.robot_fov_deg / 2.0;
  if (cfg.robot_fov_fade_deg == 0) return angle_deg <= half ? 1.0 : 0.0;
  return std::clamp((half - angle_deg) / cfg.robot_fov_fade_deg, 0.0, 1.0);
}
utils::Vector2d pair_force(const sfm::Agent & a, const sfm::Agent & b) {
  // Use the pinned LightSFM single-agent kernel. Subtract its non-social terms
  // by reading socialForce only; perturb singular inputs in this private copy.
  auto me = a; auto other = b;
  if ((other.position - me.position).norm() < 1e-6)
    other.position = me.position - direction(a, b) * 1e-6;
  auto diff = other.position - me.position;
  auto interaction = me.params.lambda * (me.velocity - other.velocity) + diff.normalized();
  if (interaction.norm() < 1e-6) other.velocity += utils::Vector2d(1e-6, 0);
  std::vector<sfm::Agent> neighbours{other};
  sfm::SFM.computeForces(me, neighbours);
  finite(me.forces.socialForce.getX()); finite(me.forces.socialForce.getY());
  return me.forces.socialForce;
}
}

using Intent = arena_multi_hunav_msgs::msg::BehaviorIntent;
using Motion = arena_multi_hunav_msgs::msg::MotionDiagnostic;
using IntentV3 = arena_multi_hunav_msgs::msg::BehaviorIntentV3;
using MotionV3 = arena_multi_hunav_msgs::msg::MotionDiagnosticV3;
static void vector_msg(geometry_msgs::msg::Vector3 & dst, const utils::Vector2d & v) {
  dst.x = v.getX(); dst.y = v.getY();
}
template<typename Plan = Intent, typename Diagnostic = Motion>
static Compute::Response integrate_impl(const Compute::Request & req, const Config & cfg,
    const std::vector<Plan> * intents, std::vector<Diagnostic> * diagnostics) {
  Compute::Response out; out.epoch = req.epoch; out.step = req.step;
  try {
    finite(req.dt); require(req.dt > 0 && req.dt <= 0.025000001, "dt outside (0, 0.025]");
    require(req.current_agents.header.frame_id == "map" && req.robots.header.frame_id == "map", "frame must be map");
    require(req.current_agents.header.stamp == req.robots.header.stamp, "snapshot timestamps differ");
    require(!req.current_agents.agents.empty(), "no pedestrians");
    for (double v : {cfg.width, cfg.height, cfg.near_gain, cfg.near_sigma, cfg.max_acceleration, cfg.max_speed}) {
      finite(v); require(v > 0, "invalid core configuration");
    }
    finite(cfg.robot_clearance); require(cfg.robot_clearance >= 0, "negative clearance");
    finite(cfg.robot_fov_deg); finite(cfg.robot_fov_fade_deg);
    require(cfg.robot_fov_deg > 0 && cfg.robot_fov_deg <= 360, "robot_fov_deg outside (0,360]");
    require(cfg.robot_fov_fade_deg >= 0 && cfg.robot_fov_fade_deg <= cfg.robot_fov_deg / 2,
      "robot_fov_fade_deg outside [0,robot_fov_deg/2]");
    std::set<int> ids; std::set<std::string> names;
    std::map<int, sfm::Agent> people, robots;
    for (const auto & a : req.current_agents.agents) {
      require(a.id > 0 && a.type == Agent::PERSON && a.behavior.type == 1, "multi_sfm requires positive-id regular people");
      require(a.group_id == -1, "group behaviors are not supported by multi_sfm v1");
      require(ids.insert(a.id).second && !a.name.empty() && names.insert(a.name).second, "duplicate identity");
      people.emplace(a.id, convert(a));
    }
    for (const auto & a : req.robots.agents) {
      require(a.id < 0 && a.type == Agent::ROBOT, "robots require negative ids");
      require(ids.insert(a.id).second && !a.name.empty() && names.insert(a.name).second, "duplicate identity");
      robots.emplace(a.id, convert(a));
    }
    std::map<int, Modifier> mods;
    for (const auto & m : req.modifiers) {
      require(people.count(m.agent_id) && mods.emplace(m.agent_id, m).second, "invalid modifier identity");
      for (double v : {m.speed_scale, m.social_scale, m.robot_scale, m.space_scale}) {
        finite(v); require(v >= 0 && v <= 4, "modifier outside [0,4]");
      }
      require(m.space_scale >= 1, "reserved space_scale outside [1,4]");
    }
    require(mods.size() == people.size(), "missing modifiers");
    std::map<int, Plan> plans;
    if (intents) {
      for (const auto & i : *intents) {
        require(people.count(i.agent_id) && plans.emplace(i.agent_id, i).second, "invalid intent identity");
        if constexpr (std::is_same_v<Plan, IntentV3>) {
          require(i.mode <= IntentV3::THREATENING, "invalid V3 intent mode");
          require(i.robot_response <= IntentV3::STATIC_OBSTACLE, "invalid robot response");
          finite(i.desired_yaw); finite(i.max_yaw_rate); finite(i.yaw_stop_speed);
          require(i.max_yaw_rate > 0 && i.max_yaw_rate <= 1 && i.yaw_stop_speed >= 0 && i.yaw_stop_speed <= .05,
            "invalid observation yaw limits");
          require(!i.override_yaw || (i.mode != IntentV3::REGULAR && i.override_desired_velocity), "invalid yaw intent");
        } else require(i.mode <= Intent::SCARED, "invalid intent mode");
        finite(i.desired_velocity_x); finite(i.desired_velocity_y);
        require(std::hypot(i.desired_velocity_x, i.desired_velocity_y) <= cfg.max_speed + 1e-9,
          "intent speed exceeds physical limit");
        require(i.has_target_robot ? robots.count(i.target_robot_id) > 0 : i.target_robot_id == 0,
          "invalid intent target");
        require(i.mode == Intent::REGULAR ||
          (i.has_target_robot && i.override_desired_velocity && !i.allow_task_progress), "invalid special intent");
        require(i.mode != Intent::REGULAR || (!i.has_target_robot && i.allow_task_progress), "invalid regular task intent");
        require(i.override_desired_velocity ||
          (i.desired_velocity_x == 0 && i.desired_velocity_y == 0 && i.mode == Intent::REGULAR), "inactive intent velocity");
      }
      require(plans.size() == people.size(), "missing intents");
    }
    out.updated_agents = req.current_agents;
    for (auto & msg : out.updated_agents.agents) {
      auto me = people.at(msg.id); const auto & m = mods.at(msg.id);
      me.desiredVelocity = std::min(cfg.max_speed, me.desiredVelocity * m.speed_scale);
      const double x = me.position.getX(), y = me.position.getY();
      require(x >= 0 && x <= cfg.width && y >= 0 && y <= cfg.height, "pedestrian outside world");
      me.obstacles1 = {{0, std::clamp(y, 0.0, cfg.height)}, {cfg.width, std::clamp(y, 0.0, cfg.height)},
        {std::clamp(x, 0.0, cfg.width), 0}, {std::clamp(x, 0.0, cfg.width), cfg.height}};
      for (auto & p : me.obstacles1) if ((p - me.position).norm() < 1e-6) p += utils::Vector2d(1e-6, 1e-6);
      std::vector<sfm::Agent> empty;
      sfm::SFM.computeForces(me, empty);
      const Plan * intent = intents ? &plans.at(msg.id) : nullptr;
      const bool override_motion = intent && intent->override_desired_velocity;
      if (override_motion) {
        me.forces.desiredForce = utils::Vector2d(intent->desired_velocity_x, intent->desired_velocity_y) - me.velocity;
        me.forces.desiredForce *= me.params.forceFactorDesired / me.params.relaxationTime;
        require(me.velocity.norm() <= cfg.max_speed + 1e-6, "input speed exceeds physical limit");
      }
      auto total = me.forces.desiredForce + me.forces.obstacleForce;
      utils::Vector2d human_social, robot_social, robot_near, robot_static;
      for (const auto & [id, other] : people) if (id != me.id) {
        const auto contribution = pair_force(me, other) * m.social_scale;
        total += contribution; human_social += contribution;
      }
      for (const auto & [id, robot] : robots) {
        const double gap = (me.position - robot.position).norm() - me.radius - robot.radius;
        const double social_weight = robot_visibility(me, robot, cfg) * m.robot_scale;
        utils::Vector2d force;
        bool static_response = false;
        if constexpr (std::is_same_v<Plan, IntentV3>)
          static_response = intent && intent->robot_response == IntentV3::STATIC_OBSTACLE;
        if (static_response) {
          // Reuse the wall obstacle kernel at the nearest circular boundary.
          // For center overlap use the outward normal directly (a boundary
          // sample inside the circle would otherwise repel inwards).
          const auto normal = direction(me, robot);
          force = normal * (me.params.forceFactorObstacle *
            std::exp(std::clamp(-gap / me.params.forceSigmaObstacle, -50.0, 10.0)));
          robot_static += force;
        } else {
          if (social_weight > 0) force = pair_force(me, robot) * social_weight;
          robot_social += force;
        }
        // Short-range geometry term is always present, including when psychology
        // suppresses social avoidance. It is not a hard collision guarantee.
        const auto near = direction(me, robot) * (cfg.near_gain * std::exp(std::clamp((cfg.robot_clearance - gap) / cfg.near_sigma, -50.0, 10.0)));
        force += near; robot_near += near;
        total += force;
        arena_multi_hunav_msgs::msg::RobotInfluence info;
        info.agent_id = me.id;
        const auto found = std::find_if(req.robots.agents.begin(), req.robots.agents.end(), [id](const auto & r){return r.id == id;});
        info.robot_name = found->name; info.force.x = force.getX(); info.force.y = force.getY(); info.clearance = gap;
        out.influences.push_back(info);
      }
      finite(total.getX()); finite(total.getY());
      Diagnostic diagnostic; diagnostic.agent_id = msg.id;
      vector_msg(diagnostic.desired_force, me.forces.desiredForce);
      vector_msg(diagnostic.obstacle_force, me.forces.obstacleForce);
      vector_msg(diagnostic.human_social_force, human_social);
      vector_msg(diagnostic.robot_social_force, robot_social);
      if constexpr (std::is_same_v<Diagnostic, MotionV3>)
        vector_msg(diagnostic.robot_static_force, robot_static);
      vector_msg(diagnostic.robot_near_force, robot_near);
      vector_msg(diagnostic.total_before_limit, total);
      diagnostic.acceleration_limited = total.norm() > cfg.max_acceleration;
      if (total.norm() > cfg.max_acceleration) total *= cfg.max_acceleration / total.norm();
      vector_msg(diagnostic.acceleration, total);
      me.forces.globalForce = total;
      const auto goals_before = me.goals.size();
      if (override_motion) {
        const auto old_yaw = me.yaw;
        me.velocity += total * req.dt;
        diagnostic.speed_limited = me.velocity.norm() > cfg.max_speed;
        if (diagnostic.speed_limited) me.velocity *= cfg.max_speed / me.velocity.norm();
        me.yaw = me.velocity.angle();
        me.position += me.velocity * req.dt;
        me.linearVelocity = me.velocity.norm();
        me.angularVelocity = (me.yaw - old_yaw).toRadian() / req.dt;
        if (intent->allow_task_progress && !me.goals.empty() &&
            (me.goals.front().center - me.position).norm() <= me.goals.front().radius) {
          auto goal = me.goals.front(); me.goals.pop_front();
          if (me.cyclicGoals) me.goals.push_back(goal);
        }
      } else {
        diagnostic.speed_limited = (me.velocity + total * req.dt).norm() > me.desiredVelocity;
        sfm::SFM.updatePosition(me, req.dt);
      }
      // LightSFM assigns atan2(0,0) when stopped; retain the last visual heading.
      if (me.velocity.getX() == 0 && me.velocity.getY() == 0) {
        me.yaw = people.at(msg.id).yaw;
        me.angularVelocity = 0;
      }
      if constexpr (std::is_same_v<Plan, IntentV3>) {
        if (intent && intent->override_yaw && me.velocity.norm() <= intent->yaw_stop_speed &&
            people.at(msg.id).velocity.norm() <= intent->yaw_stop_speed) {
          const double old_yaw = people.at(msg.id).yaw.toRadian();
          const double error = std::remainder(intent->desired_yaw - old_yaw, 2 * std::acos(-1.));
          const double turn = std::clamp(error, -intent->max_yaw_rate * req.dt, intent->max_yaw_rate * req.dt);
          me.yaw.setRadian(old_yaw + turn); me.angularVelocity = turn / req.dt;
          diagnostic.yaw_applied = true;
        }
        diagnostic.applied_yaw = me.yaw.toRadian();
        diagnostic.applied_yaw_rate = me.angularVelocity;
      }
      finite(me.position.getX()); finite(me.position.getY());
      require(me.position.getX() >= me.radius && me.position.getX() <= cfg.width - me.radius &&
        me.position.getY() >= me.radius && me.position.getY() <= cfg.height - me.radius, "wall envelope violation");
      msg.position.position.x = me.position.getX(); msg.position.position.y = me.position.getY();
      msg.yaw = me.yaw.toRadian(); msg.position.orientation.x = 0; msg.position.orientation.y = 0;
      msg.position.orientation.z = std::sin(msg.yaw / 2); msg.position.orientation.w = std::cos(msg.yaw / 2);
      msg.velocity.linear.x = me.velocity.getX(); msg.velocity.linear.y = me.velocity.getY();
      msg.velocity.angular.z = me.angularVelocity; msg.linear_vel = me.linearVelocity; msg.angular_vel = me.angularVelocity;
      // Preserve the base desired_velocity. Modifiers are applied afresh next tick.
      if (intent && !intent->allow_task_progress) {
        // Persistent task queue is unchanged throughout temporary behavior.
      } else if (me.goals.size() < goals_before) msg.goals.erase(msg.goals.begin());
      else if (msg.cyclic_goals && !msg.goals.empty() &&
          (people.at(msg.id).goals.front().center - me.position).norm() <= msg.goal_radius)
        std::rotate(msg.goals.begin(), msg.goals.begin() + 1, msg.goals.end());
      if (diagnostics) diagnostics->push_back(diagnostic);
    }
    out.updated_agents.header.stamp = (rclcpp::Time(req.current_agents.header.stamp) + rclcpp::Duration::from_seconds(req.dt));
    out.success = true;
  } catch (const std::exception & e) {
    out.success = false; out.error = e.what(); out.updated_agents = req.current_agents; out.influences.clear();
    if (diagnostics) diagnostics->clear();
  }
  return out;
}
Compute::Response integrate(const Compute::Request & req, const Config & cfg) {
  return integrate_impl<Intent, Motion>(req, cfg, nullptr, nullptr);
}
ComputeV2::Response integrate(const ComputeV2::Request & req, const Config & cfg) {
  Compute::Request base;
  base.epoch = req.epoch; base.step = req.step; base.dt = req.dt;
  base.current_agents = req.current_agents; base.robots = req.robots; base.modifiers = req.modifiers;
  ComputeV2::Response out;
  auto result = integrate_impl(base, cfg, &req.intents, &out.motion);
  out.epoch = result.epoch; out.step = result.step; out.success = result.success; out.error = result.error;
  out.updated_agents = result.updated_agents; out.influences = result.influences;
  return out;
}
ComputeV3::Response integrate(const ComputeV3::Request & req, const Config & cfg) {
  Compute::Request base;
  base.epoch = req.epoch; base.step = req.step; base.dt = req.dt;
  base.current_agents = req.current_agents; base.robots = req.robots; base.modifiers = req.modifiers;
  ComputeV3::Response out;
  auto result = integrate_impl(base, cfg, &req.intents, &out.motion);
  out.epoch = result.epoch; out.step = result.step; out.success = result.success; out.error = result.error;
  out.updated_agents = result.updated_agents; out.influences = result.influences;
  return out;
}
}
