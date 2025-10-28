#include "aiv_nav/controller.hpp"
#include "utils/utils.hpp"

#include <cmath>
#include <iostream>

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

double ModeA::calculate_control_signal(const Controller& ctrl)
{
  if (goal_flag_) {
    goal_angle_ = std::atan2(ctrl.get_closest_lidar_point()[1] - ctrl.get_robot_state()[1],
                             ctrl.get_closest_lidar_point()[0] - ctrl.get_robot_state()[0]);\
    goal_flag_ = false;
  }
  double delta_angle = std::atan2(std::sin(goal_angle_ - ctrl.get_robot_state()[2]),
                                  std::cos(goal_angle_ - ctrl.get_robot_state()[2]));;
  return ctrl.get_angular_velocity() * utils::sign(delta_angle);
}