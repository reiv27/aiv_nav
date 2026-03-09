#include "reactive_circumnav/states.hpp"

#include <chrono>
#include <cmath>
#include <iostream>

#include "reactive_circumnav/controller.hpp"
#include "utils/utils.hpp"

std::string_view ModeA::name() const
{
  return "ModeA";
}

StateName ModeA::state_name() const
{
  return StateName::ModeA;
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
  if (ctrl.get_dt() > 0.0) {
    ctrl.set_dR_dot((dR - ctrl.get_dR_prev()) / ctrl.get_dt());
  } else {
    ctrl.set_dR_dot(0.0);
  }
  ctrl.set_dR_prev(dR);

  if (goal_flag_) {
    goal_angle_ = std::atan2(ctrl.get_closest_lidar_point()[1] - ctrl.get_robot_state()[1],
                             ctrl.get_closest_lidar_point()[0] - ctrl.get_robot_state()[0]);
    goal_flag_ = false;
  }
  const double delta_angle = std::atan2(std::sin(goal_angle_ - ctrl.get_robot_state()[2]),
                                  std::cos(goal_angle_ - ctrl.get_robot_state()[2]));;
  return ctrl.get_angular_velocity() * utils::sign(delta_angle);
}