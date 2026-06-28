#include "drone/Decelerating.hpp"
#include "drone/Accelerating.hpp"
#include "drone/Turning.hpp"
#include "drone/DroneContext.hpp"

#include <memory>

namespace drone {

// adjust drone direction if needed
// calculate distance when accelerating and up-date drone position accordingly
// increase speed
// if speed reached max speed, return true (acceleration completed)
// or false - continue accelerating
std::unique_ptr<IDroneState> Decelerating::execute(DroneContext& ctx)
{
  if (ctx.execDecelerating()) {
    if (ctx.hasToTurn) {
      return std::make_unique<Turning>();
    }
    else {
      return std::make_unique<Accelerating>();
    }
  }

  return nullptr;
}

}  // namespace drone