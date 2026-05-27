#include "drone.hpp"
#include "math/point_math.hpp"
#include "math/angle_math.hpp"
#include "dto/Ammo.hpp"
#include "ballistics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using Point = pointmath::Point;
using AngleRad = anglemath::AngleRad;

/* **** drone.hpp / drone.cpp
  TargetState::
    predict target position for use in lead targeting

  Mission::
    steer drone along drop path

  DroneConfig:: for initial data from input file
  Drone::
    update drone params
    choose target
  */

namespace drone {

namespace {  // for helpers #####################################

Point getDestination(const ballistics::DropSolution& drop_route)
{
  if (drop_route.has_intermediate_point) {
    return drop_route.interm_p;
  }
  else {
    return drop_route.fire_p;
  }
}

Point getFP(const ballistics::DropSolution& drop_route)
{
  return drop_route.fire_p;
}

}  // namespace

// to be class private funcs ############
auto Drone::getAmmoFlyDist() const -> double
{
  return tgts[0].dropRoute.horizontal_fall_distance_m;  // it does not depend on target, so we can take any
};

auto Drone::getAmmoFlyTime() const -> double
{
  return tgts[0].dropRoute.fall_time_s;  // it does not depend on target, so we can take any
};

/**** returns time to turn on delta angle
 * returns 0 if absolute turn value is less than threshold  //
 */
auto Drone::getTurnTime(AngleRad delta) const -> double
{
  double abs_d = std::abs(delta.value);
  return abs_d < turnThrld ? 0.0 : abs_d / angSpeed;
}

auto Drone::getTimeToFlyToInterimPoint(double dist) const -> double
{
  // we assume, starting and final drone states are Stopped
  double cruize_dist = dist - 2.0 * accPath;

  if (cruize_dist > eps) {
    double cruizeT = cruize_dist / attSpeed;
    return cruizeT + 4.0 * accPath / attSpeed;
  }
  else {
    return 2.0 * std::sqrt(dist / kAcceleration);
  }
}

auto Drone::getTimeToFlyToFP(double dist_to_fp) const -> double
{
  // we assume, starting drone state is Stopped or Accelerating, and and final - Moving
  double time_to_accelerate = getTimeToGainAttackSpeed();
  double dist_at_acc = 0.0;
  if (state != MOVING) {
    dist_at_acc = (speed + kAcceleration * time_to_accelerate / 2.0) * time_to_accelerate;
  }

  const double cruizeDist = dist_to_fp - dist_at_acc;
  if (cruizeDist > eps) {
    return cruizeDist / attSpeed + time_to_accelerate;
  }
  return time_to_accelerate;
}

auto Drone::getMinTimeToTurn(double abs_delta_angle, double time_on_move, double time_step)
{
  double turn_on_the_move = turnThrld * floor(time_on_move / time_step);
  if (abs_delta_angle > turn_on_the_move) {  // some Turn on the spot is needed
    double min_time_to_turn = getTurnTime(abs_delta_angle - turn_on_the_move);
    return min_time_to_turn;
  }
  return 0.0;
}

// return 0 if direct manuver to FP is to continue, or eeror code
auto Drone::recalculateFPOntheRoute(double ammo_f_dist, double time_step) -> int
{
  double angle2tgt, dist2tgt;
  Point dr2tgt = mission.tgt_lead_pos - coord;
  pointmath::trxPointToDistAngle(dr2tgt, dist2tgt, angle2tgt);

  mission.destAngle = angle2tgt;  // update destination according to lead targeting

  double delta2fp = dist2tgt - ammo_f_dist;
  if (std::abs(delta2fp) <= kAccuracy_m) {  // we just missed the fire point, or we are close enough,
    // so we can try to fire on the move
    if (state != MOVING) {
      return 1;  // we will miss, because ammo ff time will be less than needed
    }
    else {  // fire!
      mission.state = FIRED;
      state = DECELERATING;  //@continue mission
      return 0;
    }
  }
  else if (delta2fp < 0) {
    return 2;  // we missed FP
  }

  double time_fly_to_FP = getTimeToFlyToFP(delta2fp);
  AngleRad delta_angle = AngleRad(angle2tgt - dirRad.value);
  double abs_delta_angle = std::abs(delta_angle.value);
  double min_time_to_turn = getMinTimeToTurn(abs_delta_angle, time_fly_to_FP, time_step);

  mission.destPoint = coord + pointmath::cossin(angle2tgt) * delta2fp;
  mission.dropPoint = mission.destPoint;
  mission.timer = min_time_to_turn + time_fly_to_FP;

  if (min_time_to_turn > 0.0) {
    if (speed > 0.0) {       // need to stop first, to turn
      state = DECELERATING;  // TODO: calculate/test mode decelerate-accelerate to give more time to turn
    }
    else {  // v=0, turn on the spot
      state = TURNING;
    }
  }
  else {  // we can continue moving
    state = speed < attSpeed ? ACCELERATING : MOVING;
  }

  if ((state != TURNING) && (delta_angle.value != 0.0)) {  // turn on the move if needed
    if (abs_delta_angle < turnThrld) {
      setDroneDirection(angle2tgt);
    }
    else {
      setDroneDirection((delta_angle.value < 0.0) ? dirRad.value - turnThrld : dirRad.value + turnThrld);
    }
  }

  return 0;
}

// calculate and update total_time for target tgt, and update time_accuracy as diff from previous calculation
// returns calculated total time value
auto Drone::calculateTimeForDropRoute(Point start, TargetState& tgt) -> double
{
  ballistics::DropSolution& drop_route = tgt.dropRoute;
  Point& pos_fire = tgt.dropRoute.fire_p;
  double total_time{};
  double time_to_interim{};

  if (drop_route.has_intermediate_point) {  // TODO: check here if we are at the Int point practically
    Point& pos_int = tgt.dropRoute.interm_p;
    double dist_to_interim = 0.0;
    double angle_to_interim = 0.0;
    pointmath::trxPointToDistAngle(pos_int - start, dist_to_interim, angle_to_interim);

    time_to_interim = getTurnTime(AngleRad(angle_to_interim - dirRad.value));  // turn from current dr.dir to IP
    time_to_interim += getTimeToFlyToInterimPoint(dist_to_interim);

    double dist_to_fire = 0.0;
    double angle_to_fire = 0.0;
    pointmath::trxPointToDistAngle(pos_fire - pos_int, dist_to_fire, angle_to_fire);

    AngleRad turn_between_legs = AngleRad(angle_to_fire - angle_to_interim);

    total_time = time_to_interim;
    total_time += getTurnTime(turn_between_legs);
    total_time += getTimeToFlyToFP(dist_to_fire);
  }
  else {
    double dist, angle;
    pointmath::trxPointToDistAngle(pos_fire - start, dist, angle);
    total_time = getTimeToFlyToFP(dist);
    total_time += getTurnTime(AngleRad(angle - dirRad.value));  // turn from current dr.dir to FP
  }
  tgt.time_accuracy = total_time - tgt.time_total;
  tgt.time_total = total_time;
  return total_time;
}

// **** calculate best target tag based on previous total time astimation
auto Drone::getBestTarget() -> int
{
  int bestTag{-1};  // there will be tag of the target with the least time to hit
  double bestTT{std::numeric_limits<double>::max()};

  ballistics::BallisticsInput input = {coord, {0, 0}, alt, attSpeed, accPath, ammo->mass, ammo->drag, ammo->lift};
  double ammo_fall_time = getAmmoFlyTime();

  for (size_t i = 0; i < nTargets; ++i) {
    TargetState& tgt = tgts[i];
    tgt.calculateBallisticSolutionAt(tgt.time_total + ammo_fall_time, input);
    double total_time = calculateTimeForDropRoute(coord, tgt);

    const double tgt_speed = tgt.getSpeed();
    if (tgt_speed > eps) {  // we shall not calculate time accuracy for still target
      const double accuracy_s = std::max(kAccuracy_m / tgt_speed, 0.1);

      int count = 0;
      constexpr int kMaxRecalculations = 20;
      // check time accuracy and maybe repeat calculations
      while (std::abs(tgt.time_accuracy) > accuracy_s &&
             count < kMaxRecalculations) {  // we are not accurate enough, so we can try to recalculate with new time estimation
        count++;
        tgt.calculateBallisticSolutionAt(total_time + ammo_fall_time, input);
        total_time = calculateTimeForDropRoute(coord, tgt);
      }

      if (count > countMaxRecalc)
        countMaxRecalc = count;
    }

    if (total_time < bestTT) {
      bestTT = total_time;
      bestTag = i;
    }
  }

  return bestTag;
}

// to be class public funcs ############
auto Drone::startNewMission(double time_step) -> int
{
  if (!((state == STOPPED) || (state == TURNING))) {  // wait till Stopped //TODO: to improve, add recalculating missions here
    state = DECELERATING;                             // @start new mission
    return -1;
  }

  kAccuracy_m = attSpeed * time_step / 2.0;

  int tgt_tag = getBestTarget();

  TargetState& tgt = tgts[tgt_tag];
  ballistics::DropSolution& drop_route = tgt.dropRoute;
  Point dest = getDestination(drop_route);

  // decide on mission state
  if (drop_route.has_intermediate_point) {  // check distance
    double dist_to_interim = pointmath::getLength(dest - coord);
    if (dist_to_interim < kAccuracy_m) {  // we are practically at Interim point
      mission.state = TO_FIREP;
      dest = getFP(drop_route);
    }
    else {
      mission.state = TO_INTERIMP;
    }
  }
  else {
    mission.state = TO_FIREP;
  }

  mission.destAngle = pointmath::getAngle(dest - coord);
  mission.destPoint = dest;

  AngleRad delta_angle_to_dest = mission.destAngle - dirRad.value;

  if (std::abs(delta_angle_to_dest.value) <= turnThrld) {  // start new mission
    if (delta_angle_to_dest.value != 0.0)
      setDroneDirection(dirRad.value + delta_angle_to_dest.value);  // turn small angle
    state = ACCELERATING;                                           // both, to interim or to fire point
  }
  else {
    state = TURNING;
  }

  if (mission.state == TO_INTERIMP) {
    const double dist_to_interim = pointmath::getLength(dest - coord);
    const Point dir_to_dest = pointmath::cossin(mission.destAngle.value);

    if (dist_to_interim > 2.0 * accPath) {
      mission.maxSpeed = attSpeed;
      mission.decelerateAtPoint = mission.destPoint - dir_to_dest * accPath;
    }
    else {
      mission.maxSpeed = std::sqrt(dist_to_interim * kAcceleration);
      mission.decelerateAtPoint = coord + dir_to_dest * (dist_to_interim / 2.0);
    }
  }
  mission.timer = tgt.time_total;
  mission.dropPoint = getFP(drop_route);
  mission.tgt_lead_pos = tgt.getLeadPosition(mission.timer + getAmmoFlyTime());  //@sm
  return tgt_tag;
};

/****
 * Change drone state according to its direction and location re destination
 * turn on the move if needed
 * returns true  if fired
 */
auto Drone::continueMission(double time_step) -> bool
{
  mission.timer -= time_step;  //@cM
  if (mission.timer < 0) {
    mission.timer = 0.0;
  }

  TargetState& currentTgt = tgts[mission.tgtTag];
  double ammo_f_dist = currentTgt.dropRoute.horizontal_fall_distance_m;
  double ammo_f_time = currentTgt.dropRoute.fall_time_s;

  mission.tgt_lead_pos = currentTgt.getLeadPosition(mission.timer + ammo_f_time);  //@cm

  switch (mission.state) {
    case TO_FIREP:
      // reevaluate fire point and turn if needed, then check if we are at fire point to fire
      if ((errcode = recalculateFPOntheRoute(ammo_f_dist, time_step)) != 0) {
        breakMission();  // mission failed, we will reevaluate target and try again
        return false;
      }

      /*Дрон досяг fire point під час розгону/гальмування — не скидає бомбу, а перезапускає місію. Можливо
      надмірно суворо: можна було б чекати MOVING замість перезапуску.

      Або додати можливість скиду на довільній швидкості. Симулятор ДЗ3 це підтримує
      TODO! Але це нечесно! заряд не долетить - хммм, подивитись баллістичне рішення для іншої швидкості? */

      break;

    case TO_INTERIMP: {
      if (state == STOPPED) {  // at interim, turn to fire point
        mission.state = TO_FIREP;
        const TargetState tgt = tgts[mission.tgtTag];
        mission.destPoint = getFP(tgt.dropRoute);
        mission.destAngle = pointmath::getAngle(mission.destPoint - coord);
        state = TURNING;  //@continue to fire point
        return false;
      }
      else if (state == ACCELERATING) {   // check max speed
        if (mission.maxSpeed < attSpeed)  // otherwise continue acccelerating to interim until Moving
        {
          double next_speed = speed + kAcceleration * time_step;
          if ((speed <= mission.maxSpeed) && (next_speed > mission.maxSpeed)) {
            // check if next step is better
            if ((mission.maxSpeed - speed) < (next_speed - mission.maxSpeed)) {  // start decelerating
              state = DECELERATING;
            }
          }
        }
      }
      else if (state == MOVING) {  // check if we are at point to start decelerating
        double dist_to_dec = pointmath::getLength(mission.decelerateAtPoint - coord);
        if (dist_to_dec <= kAccuracy_m) {  // we just missed the interim point, or we are close enough,
          state = DECELERATING;
        }
      }
      // if state == TURNING) - continue to interim until Accelerating at Move
      // if state == DECELERATING) - continue to interim  until Stopped
    } break;

    default:  // COMPLETED FAILED NONE
      throw std::runtime_error("ERR_TODO -mission state not implemented in continueMission: ");
  }

  return mission.state == FIRED;
}

// ## retuns time of ammo fly
// and  hit coord
auto Drone::getHitCoordAndAmmoFlyTime(Point& hit_coord) -> double
{
  const double ammo_fly_dist = getAmmoFlyDist();
  hit_coord = coord + dirXY * ammo_fly_dist;

  return getAmmoFlyTime();
}

/****
 * Оновити координати, швидкість та стан дрона відповідно до поточної кроку
 * NB! if Turning, check if angle to destination is reached, if yes, change state to Accelerating,
 * NB! drone state is only changes when Accelerating or Decelerating on reaching attack speed or 0-speed accordingly
 */
void Drone::moveDrone(double dt)
{
  switch (state) {
    case STOPPED:  // waiting the results of previous mission, no move, no state change
      break;
    case TURNING: {
      double dval = angSpeed * dt;
      AngleRad dturn = mission.destAngle - dirRad;
      double delta_angle = std::abs(dturn.value);
      dval = std::min(dval, delta_angle);                         // do not turn more than needed
      if ((delta_angle <= turnThrld) || (delta_angle <= dval)) {  // turn is completed
        setDroneDirection(mission.destAngle.value);
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

std::ostream& operator<<(std::ostream& os, const drone::TargetState& tgt)
{
  return os << tgt.last_known << " V" << tgt.velocity << " Drop_route: " << tgt.dropRoute << " total_s: " << tgt.time_total;
}

std::ostream& operator<<(std::ostream& os, const drone::Mission& m)
{
  if (m.tgtTag < 0)
    return os << "=>No target";

  return os << "=>T#" << m.tgtTag << " tmr=" << m.timer << " TA" << m.destAngle << " Dest" << m.destPoint << ' ' << m.missionStateToStr()
            << " TLP" << m.tgt_lead_pos;
}
auto Drone::updateSimStep(SimStep& step) const -> void
{
  step.pos = coord;                                   // позиція дрона
  step.direction = static_cast<float>(dirRad.value);  // напрямок (рад)
  step.state = state;                                 // стан автомата (0-4)
  step.targetIdx = mission.tgtTag;                    // індекс поточної цілі
  if (state == MOVING)
    step.aimPoint = coord + dirXY * getAmmoFlyDist();  // куди впаде бомба (якщо скинути зараз)
  if (mission.tgtTag >= 0) {
    step.dropPoint = mission.dropPoint;           // точка скиду (куди летить дрон)
    step.predictedTarget = mission.tgt_lead_pos;  // прогнозована позиція цілі
  }
}
}  // namespace drone
