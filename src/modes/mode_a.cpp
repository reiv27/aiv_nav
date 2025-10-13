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
  ctrl.increment_count();

  if (ctrl.count() >= 3) {
    ctrl.set_state(std::make_unique<ModeC>());
  }
}

double ModeA::calculate_control_signal(const Controller& ctrl)
{
  if (goal_flag_) {
    double azimuth = std::atan2(ctrl.get_closest_lidar_point()[1] - ctrl.get_robot_state()[1],
                                ctrl.get_closest_lidar_point()[0] - ctrl.get_robot_state()[0]);
    utils::normalize_angle(azimuth);
    double rot_angle = ctrl.get_robot_state()[2] - azimuth;
    utils::normalize_angle(rot_angle);
    goal_angle_ = rot_angle;
    goal_flag_ = false;
  }

  const double delta_angle = goal_angle_ - ctrl.get_robot_state()[2];
  return ctrl.get_angular_velocity() * utils::sign(delta_angle);
}