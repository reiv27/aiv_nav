#include "aiv_nav/controller.hpp"
#include "utils/utils.hpp"

#include <chrono>
#include <iostream>

std::string_view ModeC::name() const
{
  return "ModeC";
}

void ModeC::handle(Controller& ctrl)
{

}

double ModeC::calculate_control_signal(const Controller& ctrl)
{
  const double saturated_dR = utils::saturation(ctrl.get_dR(), -0.1, 0.1);
  const double second_part = ctrl.get_nu() * 0.025 * saturated_dR;
  const double sign = utils::sign(ctrl.get_dR_diff() + second_part);
  return ctrl.get_angular_velocity() * sign;

  // def calc_u_mode_C(self):
  //   r = np.array([self.x, self.y])
  //   dR = self.lidar.closest_distance - self.d
  //   # ddR = (dR - self.dR_last) / self.dt
  //   self.dR_last = dR

  //   sat = vec_ops.saturation(dR, -0.1, 0.1)
  //   second_part = self.n * 0.025 * sat
  //   sgn = np.sign(ddR + second_part)
  //   return self.angular_velocity * sgn
}