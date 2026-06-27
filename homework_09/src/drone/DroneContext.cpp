#include "drone/DroneContext.hpp"
// #include "dto/BallisticResult.hpp"
// #include "interfaces/IBallisticSolver.hpp"
// #include "math/angle_math.hpp"
// #include "math/point_math.hpp"

#include <cmath>
#include "math/angle_math.hpp"
#include "math/point_math.hpp"

namespace {
inline constexpr double kEps = 1e-9;
}

namespace drone {

auto DroneContext::_setDir(double aR) -> void
{
  dirRad = aR;
  dirXY = pointmath::cossin(dirRad.value);
};

auto DroneContext::_adjustDir() -> void
{
  anglemath::AngleRad delta = angleToTLpos - dirRad.value;
  if (std::fabs(delta.value) < turnThrld) {
    _setDir(angleToTLpos.value);
  }
  else {
    _setDir(delta.value < 0.0 ? dirRad.value - turnThrld : dirRad.value + turnThrld);
  }
}

auto DroneContext::updateBasicAmmoRes() -> bool
{
  if (ammo == nullptr) {
    return false;
  }

  ballResult = solver->solveAmmo(alt, attSpeed, *ammo);  // TODO check exception in Analytical solver
  ammoBaseFFTime = ballResult.ffTime;
  ammoBaseHDist = ballResult.hDist;
  return true;
}

auto DroneContext::_updateSpeedDependentCtx() -> void
{
  if (speed == attSpeed) {
    timeToGainAttSpeed = 0.0;
    distToGainAttSpeed = 0.0;
    timeToStop = kAccTime;
    distToStop = accPath;
  }
  else if (speed < kEps) {
    timeToGainAttSpeed = kAccTime;
    distToGainAttSpeed = accPath;
    timeToStop = 0.0;
    distToStop = 0.0;
  }
  else {
    timeToStop = speed / kAcceleration;
    timeToGainAttSpeed = kAccTime - timeToStop;
    distToStop = kAcceleration * timeToStop * timeToStop / 2.0;
    distToGainAttSpeed = accPath - distToStop;
    ballResult = solver->solveAmmo(alt, speed, *ammo);  // TODO check exception in Analytical solver
  }
}

auto DroneContext::setDestToFP(pointmath::Point dest) -> void
{
  destPoint = dest;
  maxSpeed = attSpeed;
  hasToTurn = false;
};

auto DroneContext::execMoving() -> bool
{
  bool done = false;
  double dist = pointmath::getLength(destPoint - coord);
  double time_to_dest;
  if (hasToTurn) {
    time_to_dest = accPath / kAcceleration;  // minimum to stop
    if (dist > accPath) {
      time_to_dest += (dist - accPath) / attSpeed;  // continue moving
    }
    else {
      done = true;  // we are approaching interim point and have to start decelerating //TODO not using interim p
    }
  }
  else {
    time_to_dest = dist / attSpeed;
  }
  anglemath::AngleRad delta = angleToTLpos - dirRad;
  double min_time_to_turn = getMinTimeToTurn(delta, time_to_dest);

  if (done || (min_time_to_turn > 0.0)) {  // start decelerating
    execDecelerating();                    
    return true;
  }

  _adjustDir();                             //@exec moving turn on the flight and if needed
  coord += dirXY * (attSpeed * kTimeStep);  //@moving
  return done;
}

auto DroneContext::execTurning() -> bool
{
  bool done = false;
  double dval = angSpeed * kTimeStep;
  anglemath::AngleRad delta = angleToTLpos - dirRad.value;

  if (std::fabs(delta.value) <= dval) {  // final turning step
    _setDir(angleToTLpos.value);
    done = true;
  }
  else {
    _setDir((delta.value < 0.0) ? dirRad.value - dval : dirRad.value + dval);
  }

  return done;
}

auto DroneContext::execDecelerating() -> bool
{
  _adjustDir();  //@exec decel
  bool done = false;

  // decrease speed. if 0, return true
  const double time_to_stop = speed / kAcceleration;
  const double step_dt = std::min(kTimeStep, time_to_stop);

  const double dist = (speed - kAcceleration * step_dt / 2.0) * step_dt;
  coord += dirXY * dist;  //@decel-ng
  speed -= kAcceleration * step_dt;

  if (time_to_stop <= kTimeStep) {  // it is last decelerating step
    speed = 0.0;                    // to avoid micro-diff
    done = true;
  }
  _updateSpeedDependentCtx();  //@decel-ng
  return done;
}

// adjust drone direction if needed
// calculate distance when accelerating and up-date drone position accordingly
// increase speed
// if speed reached max speed, return true (acceleration completed)
// or false - continue accelerating
auto DroneContext::execAccelerating() -> bool
{
  _adjustDir();  //@exec accel
  bool done = false;
  // increase speed. if attack speed, change state to Moving

  const double time_to_max_speed = (maxSpeed - speed) / kAcceleration;
  const double acc_dt = std::min(kTimeStep, time_to_max_speed);

  double dist = (speed + kAcceleration * acc_dt / 2.0) * acc_dt;  // dist with acceleration
  speed += kAcceleration * acc_dt;

  if (acc_dt < kTimeStep) {  // it is last accelerating step, some portion is in different mode
    speed = maxSpeed;        // to avoid micro-diff
    done = true;
    // next state = MOVING or DECELERATING;
    if (hasToTurn) {  // add part to fly decelerating
      speed = maxSpeed - kAcceleration * (kTimeStep - acc_dt);
      dist += (maxSpeed + speed) / 2.0 * (kTimeStep - acc_dt);
    }
    else {  // add part to fly with att speed
      dist += (kTimeStep - acc_dt) * attSpeed;
    };
  }

  coord += dirXY * dist;       //@accel-ting
  _updateSpeedDependentCtx();  //@accel-ting
  return done;
};

auto DroneContext::getTurnTime(anglemath::AngleRad delta) const -> double
{
  double abs_d = std::fabs(delta.value);
  return abs_d < turnThrld ? 0.0 : abs_d / angSpeed;
}

auto DroneContext::getMinTimeToTurn(anglemath::AngleRad delta_angle, double time_on_move) const -> double
{
  double abs_delta_angle = std::fabs(delta_angle.value);
  double turn_on_the_move = turnThrld * floor(time_on_move / kTimeStep);
  double delta = abs_delta_angle - turn_on_the_move;
  return delta > kEps ? delta / angSpeed : 0.0;
}


// based on current speed, directly (no turn on the way)
// we assume, that final drone state is Stopped
/* auto DroneContext::getTimeToFlyToInterimPoint(double dist) const -> double //TODO not used
{
  double time2stop = (attSpeed - speed) / kAcceleration;
  double dist2stop = (speed + kAcceleration * time2stop / 2.0) * time2stop;

  if (dist <= dist2stop) {
    return time2stop;
  }

  double tBefore = 0.0;
  double distBefore = 0.0;
  if (speed > kEps) {
    tBefore = speed / kAcceleration;
    distBefore = kAcceleration * tBefore * tBefore / 2.0;
  }
  dist += distBefore;

  double cruize_dist = dist - 2.0 * accPath;

  if (cruize_dist > kEps) {
    double cruizeT = cruize_dist / attSpeed;
    return cruizeT + 4.0 * accPath / attSpeed - tBefore;
  }

  return 2.0 * std::sqrt(dist / kAcceleration) - tBefore;
} */

}  // namespace drone