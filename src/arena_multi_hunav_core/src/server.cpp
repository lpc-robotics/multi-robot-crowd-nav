#include "arena_multi_hunav_core/core.hpp"
#include <rclcpp/rclcpp.hpp>
using namespace arena_multi_hunav_core;
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("multi_sfm_server");
  Config cfg;
  cfg.width = node->declare_parameter("world_width", cfg.width);
  cfg.height = node->declare_parameter("world_height", cfg.height);
  cfg.robot_clearance = node->declare_parameter("robot_clearance", cfg.robot_clearance);
  cfg.near_gain = node->declare_parameter("near_gain", cfg.near_gain);
  cfg.near_sigma = node->declare_parameter("near_sigma", cfg.near_sigma);
  cfg.max_acceleration = node->declare_parameter("max_acceleration", cfg.max_acceleration);
  cfg.max_speed = node->declare_parameter("max_speed", cfg.max_speed);
  cfg.robot_fov_deg = node->declare_parameter("robot_fov_deg", cfg.robot_fov_deg);
  cfg.robot_fov_fade_deg = node->declare_parameter("robot_fov_fade_deg", cfg.robot_fov_fade_deg);
  auto service = node->create_service<Compute>("/multirobot/hunav/compute_agents",
    [cfg](const std::shared_ptr<Compute::Request> req, std::shared_ptr<Compute::Response> res) {
      *res = integrate(*req, cfg);
    });
  rclcpp::spin(node); rclcpp::shutdown(); return 0;
}
