#include "drone/Stopped.hpp"
#include "drone/Turning.hpp"
#include "drone/Accelerating.hpp"

#include <cmath>
#include <memory>

namespace drone {

std::unique_ptr<IDroneState> Stopped::execute(DroneContext& ctx)
{
  
  ctx.updateDestDistAndDeltaAngle();

  double time_to_dest;
  if (ctx.hasToTurn) {
    time_to_dest = ctx.getTimeToFlyToInterimPoint(ctx.distToDest);
  }
  else {
    time_to_dest = ctx.getTimeToFlyToFP(ctx.distToDest);
  }
  double min_time_to_turn = ctx.getMinTimeToTurn(ctx.deltaAngle, time_to_dest);

  //not to waste time execute turning or accelerating
  if (min_time_to_turn > 0.0) {  // start turning
    ctx.execTurning(); //TODO check result ?
    return std::make_unique<Turning>();
  }

  //start accelerating
  ctx.execAccelerating();  //TODO check result ?
  return std::make_unique<Accelerating>();
}

}  // namespace drone