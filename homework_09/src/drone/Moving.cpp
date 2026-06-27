#include "drone/Moving.hpp"
#include "drone/DroneContext.hpp"
#include "drone/Decelerating.hpp"

#include <memory>

namespace drone {

std::unique_ptr<IDroneState> Moving::execute(DroneContext& ctx)
{
  if (ctx.execMoving()) {
    return std::make_unique<Decelerating>();
  }

  return nullptr;
}

}  // namespace drone