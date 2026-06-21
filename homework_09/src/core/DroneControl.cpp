#include "core/DroneControl.hpp"
#include "drone/Accelerating.hpp"

#include <cmath>
#include <memory>



namespace core {

// Оновити координати, швидкість та стан дрона відповідно до поточної кроку
auto DroneControl::move() -> void
{
  auto next = state->execute(ctx);
  if (next) {
    state = std::move(next);
  }
}

auto DroneControl::goIdle() -> void
{
  // just move for now //TODO makes sense to move away from the place of hit
  if (ctx.speed > 0.0) {                                    // continue move and turn on the move
    ctx.setDroneDirection(getDirection() + ctx.turnThrld);  // TODO set new destination angle ? change to accelerating ?
  }
  else {
    state = std::make_unique<drone::Accelerating>(); //TODO ???
  }
}

auto DroneControl::isMoving() const -> bool
{
  return ctx.attSpeed == ctx.speed;
}

//sets parameters to controll drone states at execution: dest point, hasToTurn, maxSpeed 
auto DroneControl::setDestinationPoint(pointmath::Point dest, bool has_interim_p) -> void
{
  ctx.destPoint = dest;
  ctx.maxSpeed = ctx.attSpeed;
  ctx.hasToTurn = has_interim_p;
  if (has_interim_p) { //check if max speed is less than attack speed
    double dist = pointmath::getLength(dest - ctx.coord);
    if (dist <= 2 * ctx.accPath) {
      ctx.maxSpeed = std::sqrt(dist / 2 * ctx.accPath);
    }
  }
};

}  // namespace core