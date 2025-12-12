#include "aiv_nav/states.hpp"

#include <iostream>

#include "utils/utils.hpp"
#include "aiv_nav/controller.hpp"

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
  // std::cout << "--------------------------------" << std::endl;
  // std::cout << "dR: " << dR << std::endl;
  // std::cout << "dR_prev: " << ctrl.get_dR_prev() << std::endl;
  // std::cout << "dt: " << ctrl.get_dt() << std::endl;
  const double ddR = (dR - ctrl.get_dR_prev()) / ctrl.get_dt();
  // std::cout << "ddR: " << ddR << std::endl;
  ctrl.set_dR_prev(dR);
  const double saturated_dR = utils::saturation(dR, -0.1, 0.1);
  // std::cout << "saturated_dR: " << saturated_dR << std::endl;
  const double second_part = ctrl.get_nu() * 0.025 * saturated_dR;
  // std::cout << "second_part: " << second_part << std::endl;
  const double sign = utils::sign(ddR + second_part);
  return ctrl.get_angular_velocity() * sign;
}