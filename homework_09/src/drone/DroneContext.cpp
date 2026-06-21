#include "drone/DroneContext.hpp"
// #include "math/angle_math.hpp"
// #include "math/point_math.hpp"

#include <cmath>

namespace {
inline constexpr double kEps = 1e-9;
}
/* auto DroneControl::move(double dt) -> void
{
  switch (state) {
    case STOPPED:  // waiting the results of previous mission, no move, no state change
      break;

    case TURNING: {
      double dval = angSpeed * dt;
      anglemath::AngleRad dturn = destAngle - dirRad;
      double delta_angle = std::abs(dturn.value);
      dval = std::min(dval, delta_angle);                         // do not turn more than needed
      if ((delta_angle <= turnThrld) || (delta_angle <= dval)) {  // turn is completed
        setDroneDirection(destAngle.value);
        state = ACCELERATING;
        break;
      }

      setDroneDirection((dturn.value < 0.0) ? dirRad.value - dval : dirRad.value + dval);
    } break;

    case MOVING:  // one step in the same direction
      coord += dirXY * (attSpeed * dt);
      break;

    case ACCELERATING: {  // increase speed. if attack speed, change state to Moving
      const double time_to_att_speed = (attSpeed - speed) / kAcceleration;
      const double acc_dt = std::min(dt, time_to_att_speed);

      double dist = (speed + kAcceleration * acc_dt / 2.0) * acc_dt;
      speed += kAcceleration * acc_dt;

      if (time_to_att_speed <= dt) {  // it is last accelerating step
        speed = attSpeed;
        state = MOVING;
        dist += (dt - acc_dt) * attSpeed;
      }

      coord += dirXY * dist;
      break;
    }

    case DECELERATING: {  // decrease speed. if 0, change state to Stopped
      const double time_to_stop = speed / kAcceleration;
      const double step_dt = std::min(dt, time_to_stop);

      const double dist = (speed - kAcceleration * step_dt / 2.0) * step_dt;
      coord += dirXY * dist;
      speed -= kAcceleration * step_dt;

      if (time_to_stop <= dt) {
        speed = 0.0;
        state = STOPPED;
      }
      break;
    }
  }
} */

namespace drone {
auto DroneContext::execMoving() -> bool
{
  bool done = false;
  
  double time_to_dest;
  if (hasToTurn) {
    time_to_dest = accPath / kAcceleration; //minimum to stop
    if (distToDest > accPath) {
      time_to_dest += (distToDest - accPath) / attSpeed; //continue moving
    } else {
      done = true; //we are approaching interim point and have to start decelerating
    }
    
  } else {
     time_to_dest = distToDest / attSpeed;
  }
  
  double min_time_to_turn = getMinTimeToTurn(deltaAngle, time_to_dest);

  if (done || (min_time_to_turn > 0.0)) {  // start decelerating
    adjustDirection(); //turn on the flight and if needed
    execDecelerating(); //TODO do we need result?
    return true;
  }

  coord += dirXY * (attSpeed * deltaTime);
  return done;
}

auto DroneContext::execTurning() -> bool
{
  bool done = false;
  double dval = angSpeed * deltaTime;
  if (std::fabs(deltaAngle.value) <= dval) {  // final turning step
    setDroneDirection(destAngle.value);
    done = true;
  }
  else {
    setDroneDirection((deltaAngle.value < 0.0) ? dirRad.value - dval : dirRad.value + dval);
  }

  return done;
}

auto DroneContext::execDecelerating() -> bool
{
  adjustDirection();
  bool done = false;

  // decrease speed. if 0, return true
  const double time_to_stop = speed / kAcceleration;
  const double step_dt = std::min(deltaTime, time_to_stop);

  const double dist = (speed - kAcceleration * step_dt / 2.0) * step_dt;
  coord += dirXY * dist;
  speed -= kAcceleration * step_dt;

  if (time_to_stop <= deltaTime) {  // it is last decelerating step
    speed = 0.0;                    // to avoid micro-diff
    done = true;
  }

  return done;
}

// adjust drone direction if needed
// calculate distance when accelerating and update drone position accordingly
// increase speed
// if speed reached max speed, return true (acceleration completed)
// or false - continue accelerating
auto DroneContext::execAccelerating() -> bool
{
  adjustDirection();
  bool done = false;
  // increase speed. if attack speed, change state to Moving

  const double time_to_max_speed = (maxSpeed - speed) / kAcceleration;
  const double acc_dt = std::min(deltaTime, time_to_max_speed);

  double dist = (speed + kAcceleration * acc_dt / 2.0) * acc_dt;  // dist with acceleration
  speed += kAcceleration * acc_dt;

  if (acc_dt < deltaTime) {  // it is last accelerating step, some portion is in different mode
    speed = maxSpeed;        // to avoid micro-diff
    done = true;
    // next state = MOVING or DECELERATING;
    if (hasToTurn) {  // add part to fly decelerating
      speed = maxSpeed - kAcceleration * (deltaTime - acc_dt);
      dist += (maxSpeed + speed) / 2.0 * (deltaTime - acc_dt);
    }
    else {  // add part to fly with att speed
      dist += (deltaTime - acc_dt) * attSpeed;
    };
  }

  coord += dirXY * dist;
  return done;
};

auto DroneContext::getTurnTime(anglemath::AngleRad delta) const -> double
{
  double abs_d = std::abs(delta.value);
  return abs_d < turnThrld ? 0.0 : abs_d / angSpeed;
}

auto DroneContext::getMinTimeToTurn(anglemath::AngleRad delta_angle, double time_on_move) const -> double
{
  double abs_delta_angle = std::abs(delta_angle.value);
  double turn_on_the_move = turnThrld * floor(time_on_move / deltaTime);
  if (abs_delta_angle > turn_on_the_move) {  // some Turn on the spot is needed
    double min_time_to_turn = getTurnTime(abs_delta_angle - turn_on_the_move);
    return min_time_to_turn;
  }
  return 0.0;
}

auto DroneContext::getTimeToFlyToFP(double dist_to_fp) const -> double
{
  // we assume, starting drone state is Stopped or Accelerating, and and final - Moving
  double time_to_accelerate = getTimeToGainAttackSpeed();
  double dist_at_acc = 0.0;
  if (speed != attSpeed) {
    dist_at_acc = (speed + kAcceleration * time_to_accelerate / 2.0) * time_to_accelerate;
  }

  const double cruizeDist = dist_to_fp - dist_at_acc;
  return cruizeDist / attSpeed + time_to_accelerate;
}

auto DroneContext::getTimeToFlyToInterimPoint(double dist) const -> double
{
  // we assume, starting and final drone states are Stopped
  double cruize_dist = dist - 2.0 * accPath;

  if (cruize_dist > kEps) {
    double cruizeT = cruize_dist / attSpeed;
    return cruizeT + 4.0 * accPath / attSpeed;
  }

  return 2.0 * std::sqrt(dist / kAcceleration);
}

}  // namespace drone