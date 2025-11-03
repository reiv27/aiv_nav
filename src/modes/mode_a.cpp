#include "aiv_nav/states.hpp"

#include <cmath>
#include <chrono>
#include <iostream>

#include "utils/utils.hpp"
#include "aiv_nav/controller.hpp"

std::string_view ModeA::name() const
{
  return "ModeA";
}

void ModeA::handle(Controller& ctrl)
{
  if (ctrl.get_min_dist() < (ctrl.get_rho_0() + ctrl.get_R_min())) {
    ctrl.set_state(std::make_unique<ModeC>());
  }
}

double ModeA::calculate_control_signal(Controller& ctrl)
{
  const double dR = ctrl.get_min_dist() - ctrl.get_rho_0();
  ctrl.set_dR_prev(dR);
  
  if (goal_flag_) {  
    // const std::vector<double> v0{3.73334, 1.62868};
    // const std::vector<double> v1{4.11232, -2.37539};
    // const std::vector<double> v2{4.07849, -2.54617};
    // const std::vector<double> point1{3.23023, -2.21013};
    // std::cout << utils::is_point_in_angle(v0, v1, v2, point1) << std::endl;
    // std::cout << utils::is_point_in_angle(v0, v2, v1, point1) << std::endl;
    
    goal_angle_ = std::atan2(ctrl.get_closest_lidar_point()[1] - ctrl.get_robot_state()[1],
                             ctrl.get_closest_lidar_point()[0] - ctrl.get_robot_state()[0]);
    goal_flag_ = false;
  }
  const double delta_angle = std::atan2(std::sin(goal_angle_ - ctrl.get_robot_state()[2]),
                                  std::cos(goal_angle_ - ctrl.get_robot_state()[2]));;
  return ctrl.get_angular_velocity() * utils::sign(delta_angle);
}