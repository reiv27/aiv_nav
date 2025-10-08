#include <iostream>
#include "aiv_nav/controller.hpp"

std::string_view ModeC::name() const
{
  return "ModeC";
}

void ModeC::handle(Controller& ctrl)
{
  ctrl.increment_count();

  if (ctrl.count() >= 9) {
    ctrl.set_state(std::make_unique<ModeA>());
  }
}