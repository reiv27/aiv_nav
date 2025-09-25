#include <iostream>
#include "aiv_nav/controller.hpp"

std::string_view ModeA::name() const
{
  return "ModeA";
}

void ModeA::handle(Controller& ctx)
{
  ctx.incrementCount();

  if (ctx.count() >= 3) {
    ctx.setState(std::make_unique<ModeG>());
  }
}