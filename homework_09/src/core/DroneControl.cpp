#include "core/DroneControl.hpp"
#include "math/point_math.hpp"
#include "math/angle_math.hpp"
#include <cmath>

namespace {
  inline constexpr double kEps = 1e-9;
}


namespace core {

auto DroneControl::getTimeToGainAttackSpeed() const -> double
{
  return (state == MOVING) ? 0.0 : (attSpeed - speed) / kAcceleration;
}

auto DroneControl::getMinTimeToTurn(anglemath::AngleRad delta_angle, double time_on_move, double time_step) const -> double
{
  double abs_delta_angle = std::abs(delta_angle.value);
  double turn_on_the_move = turnThrld * floor(time_on_move / time_step);
  if (abs_delta_angle > turn_on_the_move) {  // some Turn on the spot is needed
    double min_time_to_turn = getTurnTime(abs_delta_angle - turn_on_the_move);
    return min_time_to_turn;
  }
  return 0.0;
}

auto DroneControl::getTurnTime(anglemath::AngleRad delta) const -> double
{
  double abs_d = std::abs(delta.value);
  return abs_d < turnThrld ? 0.0 : abs_d / angSpeed;
}

auto DroneControl::getTimeToFlyToInterimPoint(double dist) const -> double
{
  // we assume, starting and final drone states are Stopped
  double cruize_dist = dist - 2.0 * accPath;

  if (cruize_dist > kEps) {
    double cruizeT = cruize_dist / attSpeed;
    return cruizeT + 4.0 * accPath / attSpeed;
  }

  return 2.0 * std::sqrt(dist / kAcceleration);
}

auto DroneControl::getTimeToFlyToFP(double dist_to_fp) const -> double
{
  // we assume, starting drone state is Stopped or Accelerating, and and final - Moving
  double time_to_accelerate = getTimeToGainAttackSpeed();
  double dist_at_acc = 0.0;
  if (state != MOVING) {
    dist_at_acc = (speed + kAcceleration * time_to_accelerate / 2.0) * time_to_accelerate;
  }

  const double cruizeDist = dist_to_fp - dist_at_acc;
  return cruizeDist / attSpeed + time_to_accelerate;
}

/****
 * Оновити координати, швидкість та стан дрона відповідно до поточної кроку
 * NB! if Turning, check if angle to destination is reached, if yes, change state to Accelerating,
 * NB! drone state is only changes when Accelerating or Decelerating on reaching attack speed or 0-speed accordingly
 */
auto DroneControl::move(double dt) -> void
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
}

void DroneControl::setDroneDirection(double aR)
{
  dirRad = aR;
  dirXY = pointmath::cossin(dirRad.value);
};

auto DroneControl::droneStateToStr() const -> const char*
{
  switch (state) {
    case STOPPED:
      return "STOPPED";
    case ACCELERATING:
      return "ACCELERATING";
    case DECELERATING:
      return "DECELERATING";
    case TURNING:
      return "TURNING";
    case MOVING:
      return "MOVING";
    default:
      return "UNKNOWN_STATE";
  }
}

}  // namespace core