// CPU force decomposition via actual integrate() calls; no ROS node or robot control.
#include "arena_multi_hunav_core/core.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace arena_multi_hunav_core;
using Agent = hunav_msgs::msg::Agent;

Compute::Request request(double distance, double speed, double scale, const std::vector<double> & angles) {
  Compute::Request q; q.dt = .001; q.epoch = 1; q.step = 1;
  q.current_agents.header.frame_id = "map"; q.robots.header.frame_id = "map";
  Agent p; p.id = 1; p.name = "person"; p.type = Agent::PERSON; p.group_id = -1;
  p.radius = .4; p.desired_velocity = 1; p.goal_radius = .3; p.behavior.type = 1;
  p.behavior.goal_force_factor = 2; p.behavior.obstacle_force_factor = 10; p.behavior.social_force_factor = 5;
  p.position.position.x = 15; p.position.position.y = 11.5; p.position.orientation.w = 1;
  p.velocity.linear.x = speed;
  geometry_msgs::msg::Pose goal; goal.position.x = 25; goal.position.y = 11.5; goal.orientation.w = 1;
  p.goals.push_back(goal); q.current_agents.agents.push_back(p);
  arena_multi_hunav_msgs::msg::BehaviorModifiers m; m.agent_id = 1; m.robot_scale = scale;
  q.modifiers.push_back(m);
  for (double angle : angles) {
    Agent r; r.id = -int(q.robots.agents.size())-1; r.name = "robot_" + std::to_string(-r.id);
    r.type = Agent::ROBOT; r.group_id = -1; r.radius = .35; r.position.orientation.w = 1;
    const double theta = angle * std::acos(-1.0) / 180;
    r.position.position.x = 15 + distance * std::cos(theta);
    r.position.position.y = 11.5 + distance * std::sin(theta);
    q.robots.agents.push_back(r);
  }
  return q;
}

Compute::Response compute(const Compute::Request & q, const Config & cfg) {
  auto out = integrate(q, cfg);
  if (!out.success) throw std::runtime_error(out.error);
  if (out.updated_agents.agents[0].linear_vel >= 1) throw std::runtime_error("speed cap masks acceleration");
  return out;
}

int main(int argc, char ** argv) {
  Config cfg;
#ifndef BASELINE_CORE
  if (argc > 1 && std::string(argv[1]) == "--full") cfg.robot_fov_deg = 360;
#else
  (void)argc; (void)argv;
#endif
  std::cout << std::setprecision(17)
    << "layout,initial_speed,distance,robot_scale,robots,near_sum_magnitudes,near_x,near_y,social_x,social_y,robot_x,robot_y,raw_ax,raw_ay,actual_ax,actual_ay,saturated\n";
  const std::vector<std::pair<std::string, std::vector<double>>> layouts = {
    {"none", {}}, {"front", {0}}, {"rear", {180}}, {"side", {90}}, {"edge", {95}}, {"blind", {110}},
    {"dual_front", {-30, 30}}, {"dual_rear", {-150, 150}}, {"front_rear", {0, 180}}, {"dual_edge", {-95, 95}}};
  for (const auto & layout : layouts) for (double speed : {0.0, .8})
    for (int i = 0; i <= 50; ++i) for (double scale : {0.0, 1.0, 4.0}) {
      const double distance = .75 + .025*i;
      auto q = request(distance, speed, scale, layout.second);
      auto normal = compute(q, cfg);
      auto near_q = q; near_q.modifiers[0].robot_scale = 0;
      auto near = compute(near_q, cfg);
      Config uncapped = cfg; uncapped.max_acceleration = 1e6;
      auto raw = compute(q, uncapped);
      double nx = 0, ny = 0, fx = 0, fy = 0, near_magnitudes = 0;
      for (const auto & f : near.influences) {
        nx += f.force.x; ny += f.force.y;
        const double expected = cfg.near_gain * std::exp((cfg.robot_clearance-f.clearance)/cfg.near_sigma);
        if (std::abs(std::hypot(f.force.x, f.force.y)-expected) > 1e-9)
          throw std::runtime_error("near ablation differs from physical formula");
        near_magnitudes += std::hypot(f.force.x, f.force.y);
      }
      for (const auto & f : normal.influences) {fx += f.force.x; fy += f.force.y;}
      const auto rv = raw.updated_agents.agents[0].velocity.linear;
      const auto av = normal.updated_agents.agents[0].velocity.linear;
      const double ax = (rv.x-speed)/q.dt, ay = rv.y/q.dt;
      std::cout << layout.first << ',' << speed << ',' << distance << ',' << scale << ',' << layout.second.size()
        << ',' << near_magnitudes << ',' << nx << ',' << ny << ',' << fx-nx << ',' << fy-ny
        << ',' << fx << ',' << fy << ',' << ax << ',' << ay
        << ',' << (av.x-speed)/q.dt << ',' << av.y/q.dt << ',' << (std::hypot(ax,ay) > cfg.max_acceleration) << '\n';
    }
}
