#include "reactive_circumnav/states.hpp"

#include <chrono>
#include <iostream>

#include "reactive_circumnav/controller.hpp"
#include "utils/utils.hpp"

std::string_view ModeC::name() const
{
  return "ModeC";
}

StateName ModeC::state_name() const
{
  return StateName::ModeC;
}

void ModeC::handle(Controller& ctrl)
{ 
  if (ctrl.get_disk_min_ray_length() <= utils::norm2(ctrl.get_disk_pose(),
                                                     ctrl.get_closest_lidar_point())+0.05) {
    ctrl.set_verA(ctrl.get_disk_pose());
    ctrl.set_gap_points(ctrl.get_closest_lidar_point(),
                        ctrl.get_lidar_points()[ctrl.get_disk_min_arg()]);
    ctrl.set_state(std::make_unique<ModeG>());
  }
}

double ModeC::calculate_control_signal(Controller& ctrl)
{
  const double dR = ctrl.get_min_dist() - ctrl.get_rho_0();
  const double ddR = (dR - ctrl.get_dR_prev()) / ctrl.get_dt();
  ctrl.set_dR_prev(dR);
  const double saturated_dR = utils::saturation(dR, -0.1, 0.1);
  const double second_part = ctrl.get_nu() * 0.025 * saturated_dR;
  // const double sign = utils::sign(ddR + second_part);
  const double sign = utils::soft_sign(ddR + second_part, 0.0);
  return ctrl.get_angular_velocity() * sign;
}