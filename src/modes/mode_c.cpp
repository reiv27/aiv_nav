#include <iostream>
#include "aiv_nav/controller.hpp"

std::string_view ModeC::name() const
{
  return "ModeC";
}

void ModeC::handle(Controller& ctx)
{
  ctx.incrementCount();

  if (ctx.count() >= 9) {
    ctx.setState(std::make_unique<ModeA>());
  }
}