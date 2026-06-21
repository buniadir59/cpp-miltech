#include "drone/Turning.hpp"
#include "drone/Accelerating.hpp"
#include "drone/DroneContext.hpp"

#include <memory>

namespace drone {

// NB! if Turning, check if angle to destination is reached, if yes, change state to Accelerating,
std::unique_ptr<IDroneState> Turning::execute(DroneContext& ctx)
{
  ctx.updateDestDistAndDeltaAngle();

  if (ctx.execTurning()) {
    return std::make_unique<Accelerating>();
  }

  return nullptr;
}

}  // namespace drone