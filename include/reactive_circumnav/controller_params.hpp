#ifndef REACTIVE_CIRCUMNAV_CONTROLLER_PARAMS_HPP
#define REACTIVE_CIRCUMNAV_CONTROLLER_PARAMS_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace reactive_circumnav
{

/**
 * @brief Parameters required to construct Controller
 */
struct ControllerParams
{
  std::string mode_c_control_type{"relay"};
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double rho_0{0.0};
  double R_epsilon{0.0};
  double R_vis{0.0};
  int resolution{0};
  double lidar_angle_offset{0.0};
  int window_size{0};
  double nu{1.0};
  int history_size{0};
  int curvature_points_size{0};
  double k1{1.0};
  double k2{1.0};
};

/**
 * @brief Declare and read controller parameters from node
 * @param node ROS2 node (for declare_parameter / get_parameter)
 * @return ControllerParams filled from node parameters
 */
inline ControllerParams load_controller_params(rclcpp::Node* node)
{
  node->declare_parameter<std::string>("mode_c_control_type", "relay");
  node->declare_parameter<double>("linear_velocity", 1.0);
  node->declare_parameter<double>("angular_velocity", 2.0 / 3.0);
  node->declare_parameter<double>("rho_0", 2.0);
  node->declare_parameter<double>("R_epsilon", 0.0);
  node->declare_parameter<double>("R_vis", 12.0);
  node->declare_parameter<int>("resolution", 360);
  node->declare_parameter<double>("lidar_angle_offset", 3.14159);
  node->declare_parameter<int>("window_size", 10);
  node->declare_parameter<double>("nu", 100.0);
  node->declare_parameter<int>("history_size", 5);
  node->declare_parameter<int>("curvature_points_size", 10);
  node->declare_parameter<double>("k1", 1.0);
  node->declare_parameter<double>("k2", 1.0);

  ControllerParams p;
  p.mode_c_control_type = node->get_parameter("mode_c_control_type").as_string();
  p.linear_velocity = node->get_parameter("linear_velocity").as_double();
  p.angular_velocity = node->get_parameter("angular_velocity").as_double();
  p.rho_0 = node->get_parameter("rho_0").as_double();
  p.R_epsilon = node->get_parameter("R_epsilon").as_double();
  p.R_vis = node->get_parameter("R_vis").as_double();
  p.resolution = node->get_parameter("resolution").as_int();
  p.lidar_angle_offset = node->get_parameter("lidar_angle_offset").as_double();
  p.window_size = node->get_parameter("window_size").as_int();
  p.nu = node->get_parameter("nu").as_double();
  p.history_size = node->get_parameter("history_size").as_int();
  p.curvature_points_size = node->get_parameter("curvature_points_size").as_int();
  p.k1 = node->get_parameter("k1").as_double();
  p.k2 = node->get_parameter("k2").as_double();
  return p;
}

}  // namespace reactive_circumnav

#endif  // REACTIVE_CIRCUMNAV_CONTROLLER_PARAMS_HPP
