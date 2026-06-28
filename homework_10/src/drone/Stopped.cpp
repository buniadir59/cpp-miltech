#include "drone/Stopped.hpp"
#include "drone/Turning.hpp"
#include "drone/Accelerating.hpp"
#include "drone/DroneContext.hpp"

#include <cmath>
#include <memory>

namespace drone {

std::unique_ptr<IDroneState> Stopped::execute(DroneContext& ctx)
{
  anglemath::AngleRad delta = ctx.angleToTLpos - ctx.dirRad;
  double time_to_turn = ctx.getTurnTime(delta);
  // at once execute turning or accelerating
  if (time_to_turn >= ctx.kAccTime) {  // start turning
    ctx.execTurning();      
    return std::make_unique<Turning>();
  }

  // start accelerating
  ctx.execAccelerating();
  return std::make_unique<Accelerating>();
}

}  // namespace drone