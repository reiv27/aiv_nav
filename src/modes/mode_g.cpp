#include "reactive_circumnav/states.hpp"

#include <iostream>

#include "utils/utils.hpp"
#include "reactive_circumnav/controller.hpp"

std::string_view ModeG::name() const
{
  return "ModeG";
}

StateName ModeG::state_name() const
{
  return StateName::ModeG;
}

void ModeG::handle(Controller& ctrl)
{
  bool is_robot_in_verA = utils::is_point_in_angle(ctrl.get_verA(),
                                                   ctrl.get_gap_point_1(),
                                                   ctrl.get_gap_point_2(),
                                                   ctrl.get_robot_state());
  if (!is_robot_in_verA) {
    ctrl.set_state(std::make_unique<ModeC>());
  }
}

double ModeG::calculate_control_signal(Controller& ctrl)
{
  const double dR = ctrl.get_R_min() - utils::norm2(ctrl.get_verA(), ctrl.get_robot_state());
  const double ddR = (dR - ctrl.get_dR_prev()) / ctrl.get_dt();
  ctrl.set_dR_prev(dR);
  const double saturated_dR = utils::saturation(dR, -0.1, 0.1);
  const double second_part = ctrl.get_nu() * 0.025 * saturated_dR;
  const double sign = utils::sign(ddR + second_part);
  return ctrl.get_angular_velocity() * sign;
}