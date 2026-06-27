#include "drone/Accelerating.hpp"
#include "drone/Decelerating.hpp"
#include "drone/Moving.hpp"
#include "drone/DroneContext.hpp"
#include <memory>

namespace drone {

std::unique_ptr<IDroneState> Accelerating::execute(DroneContext& ctx)
{
  if (ctx.execAccelerating()) {  // up-date position and direction and return true if completed
    if (ctx.hasToTurn) {
      return std::make_unique<Decelerating>();
    }

    return std::make_unique<Moving>();
  }

  return nullptr;
}

}  // namespace drone