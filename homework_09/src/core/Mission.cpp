#include "core/Mission.hpp"
#include "core/TargetControl.hpp"
#include "core/DroneControl.hpp"
#include "dto/BallisticResult.hpp"
#include "dto/Target.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "config/defines.hpp"
#include "math/point_math.hpp"
#include "math//angle_math.hpp"

#include <cmath>

using Point = pointmath::Point;
using AngleRad = anglemath::AngleRad;

namespace {
constexpr double kEps = 1e-9;
constexpr int kMaxRecalculations = 6;  // for drop route
}  // namespace

namespace core {

auto Mission::setSolver(IBallisticSolver* s) -> void
{
  solver = s;
  solveDropRoute();  // to get ammo FF time an dist
}

auto Mission::init(const dto::MissionConfig* mconf, DroneControl* drone_ptr, const dto::Ammo* ammo) -> void
{
  drone = drone_ptr;
  time_step = mconf->time_step;
  tgtTimeStep = mconf->tgt_time_step;
  kAccuracy_m = mconf->time_step * mconf->attack_speed / 2.0;
  dto::BallisticResult res = solver->solveAmmo(mconf->altitude, mconf->attack_speed, *ammo);
  dropRoute = solver->solve(mconf->drone_position,  // TODO remove
                            tgt_lead_pos,
                            mconf->altitude,
                            mconf->attack_speed,
                            mconf->acceleration_path,
                            *ammo);  // get ballistic solution for the first time to be able to call short version
  ammoFlyTime = res.ffTime;          // dropRoute.fall_time_s;
  ammoHorizDist = res.hDist;         // dropRoute.horizontal_fall_distance_m;
}

// target is selected (=current target tag)
// return false, if impossible to reach target
auto Mission::startNewMission(TargetControl& tgt) -> bool
{
  currTgt = &tgt;
  timer = 0;
  missionResultCode = 0;
  if (calculateMission()) {  //@snm
    drone->setDestinationAngle(destAngle.value);
    drone->setDestinationPoint(destPoint, dropRoute.has_intermediate_point);
    return true;
  }
  return false;  //@snm
};

/****
 * Change drone state according to its direction and location re destination
 * turn on the move if needed
 * returns true  if fired
 */
auto Mission::continueMission() -> bool
{
  timer -= time_step;  //@cM
  if (timer < 0) {
    timer = 0.0;
  }

  if (state == TO_INTERIMP) {
    if (!calculateMission()) {  //@cm
      breakMission();
      return false;
    };
  }

  dto::Target& target = currTgt->now;

  switch (state) {
    case TO_FIREP:
      // reevaluate fire point and turn if needed, then check if we are at fire point to fire
      missionResultCode = recalculateFPOntheRoute(target);
      if (missionResultCode > 0) {  // error
        breakMission();             //@cm // mission failed, we will reevaluate target and try again
        return false;
      }

      return missionResultCode == -1;  // fired or continue
      break;

    default:
      throw std::runtime_error("ERR_TODO -mission state not implemented in continueMission: ");
  }

  return false;
}

// return -1 if fired, 0 if direct manuver to FP is to continue, or error code
auto Mission::recalculateFPOntheRoute(dto::Target& target_coord) -> int
{
  int count = 0;
  double timeFPAccuracy = 0.0;
  while (++count < kMaxRecalculations) {
    double timeToFP = recalculateTimeToFP(target_coord);

    timeFPAccuracy = timeToFP - timer;
    timer = timeToFP;
    if (std::abs(timeFPAccuracy) < currTgt->getAccuracyS(kAccuracy_m)) {
      break;  // accuracy is ok
    }
  }

  if (count == kMaxRecalculations) {  // mission impossible
    DEBUG("MISSION IMPOSSIBLE! timer=" << timer);
    currTgt->state = core::UNREACHABLE;
    return 3;
  }

  double angle2tgt{0.0};
  double dist2tgt{0.0};
  pointmath::trxPointToDistAngle(tgt_lead_pos - drone->getPosition(), dist2tgt, angle2tgt);

  double delta2fp = dist2tgt - ammoHorizDist;
  DEBUG("@RCFP: M>" << missionStateToStr() << "@ tmr=" << timer << "@ MdA>>" << destAngle << "@MdP@" << destPoint << "@ FP@" << dropPoint
                    << "@LP@" << tgt_lead_pos << "@c=" << count << "@delta2fp=" << delta2fp);
 
  if (std::abs(delta2fp) <= kAccuracy_m) {  // we are close to the fire point

    // so we can try to fire on the move
    if (!drone->isMoving()) {  // TODO recalculate drop solution for current speed
      LOG("@at FP: not MOVING!@dist to FP=" << delta2fp << "@ tmr=" << timer);
      currTgt->state = core::UNREACHABLE;
      return 1;  // we will miss, because ammo ff time will be less than needed
    }
    // fire!
    // drone->startDecelerating();  //@recalc on the route TODO remove ?
    // TODO set dest angle and point opposite to the current to start turning after the fire ???
    return -1;  // fire
  }

  if (delta2fp < 0) {
    LOG("@Missed FP!@dist to FP=" << delta2fp);
    currTgt->state = core::UNREACHABLE;
    return 2;
  }

  //continue flying to FP
  double time_fly_to_FP = drone->getTimeToFlyToFP(delta2fp);
  if (time_fly_to_FP - timer > currTgt->getAccuracyS(kAccuracy_m)) {
    //TODO recalculate based on updated lead targeting
  }

  drone->setDestinationAngle(angle2tgt);  // destAngle = angle2tgt;  // update destination according to lead targeting
  dropPoint = drone->getPosition() + pointmath::cossin(angle2tgt) * delta2fp;
  drone->setDestinationPoint(dropPoint, false); 


  
  auto deltaAngle = AngleRad(angle2tgt - drone->getDirection());
  double min_time_to_turn = drone->getMinTimeToTurn(deltaAngle, time_fly_to_FP);




  if (min_time_to_turn > 0.0) {     // TODO move to drone state
    if (drone->getSpeed() > 0.0) {  // not enough time to turn fully on the route
      drone->startDecelerating();   // TODO move to drone state
    }
    else {                    // v=0, turn on the spot
      drone->startTurning();  // TODO move to drone state
    }
  }
  else {  // we can continue moving
    // TODO if really needed ??? //drone->state = drone->speed < drone->attSpeed ? core::ACCELERATING : core::MOVING;
  }

  if ((!drone->isTurning()) && (deltaAngle.value != 0.0)) {  // turn on the move if needed //TODO move to drone state
    double delta_threshold = drone->getTurnThreshold();
    if (std::abs(deltaAngle.value) < delta_threshold) {
      drone->setDroneDirection(angle2tgt);  //@RCFP
    }
    else {
      double curr_dir = drone->getDirection();
      drone->setDroneDirection((deltaAngle.value < 0.0) ? curr_dir - delta_threshold  //@RCFP
                                                        : curr_dir + delta_threshold);
    }
  }

  return 0;  // we shall continue mission
}

auto Mission::solveDropRoute() -> void  // wrapper for solve
{
  dropRoute = solver->solve(drone->getPosition(), tgt_lead_pos);
  ammoFlyTime = dropRoute.fall_time_s;
  ammoHorizDist = dropRoute.horizontal_fall_distance_m;
}

auto Mission::getTargetLeadPosition(const dto::Target& tgt, double deltaT) const -> pointmath::Point
{
  return tgt.position + tgt.delta * (deltaT / tgtTimeStep);
}

auto Mission::recalculateTimeToFP(dto::Target& target) -> double
{                                                                     // used in cycle in recalculate FPOntheRoute
  tgt_lead_pos = getTargetLeadPosition(target, timer + ammoFlyTime);  //@rcfp
  double angle2tgt{0.0};
  double dist2tgt{0.0};
  pointmath::trxPointToDistAngle(tgt_lead_pos - drone->getPosition(), dist2tgt, angle2tgt);
  destAngle = angle2tgt;  // update destination according to lead targeting
  double delta2fp = dist2tgt - ammoHorizDist;
  double min_time_to_turn = 0.0;
  double time_fly_to_FP = delta2fp <= kAccuracy_m ? 0.0 : drone->getTimeToFlyToFP(delta2fp);
  if (!drone->isTurning()) {
    min_time_to_turn = drone->getMinTimeToTurn(AngleRad(angle2tgt - drone->getDirection()), time_fly_to_FP);
  }

  return min_time_to_turn + time_fly_to_FP;
}

auto Mission::calculateTimeForDropRoute(const pointmath::Point& start) -> double
{
  const Point& pos_fire = dropRoute.fire_p;
  double total_time{};
  double time_to_interim{};

  if (dropRoute.has_intermediate_point) {
    double dist_to_interim = 0.0;
    double angle_to_interim = 0.0;
    pointmath::trxPointToDistAngle(dropRoute.interm_p - start, dist_to_interim, angle_to_interim);

    // check distance
    if (dist_to_interim <= kAccuracy_m) {  // we are practically at interim point, so proceed to FP from here
      dropRoute.has_intermediate_point = false;
    }
    else {
      time_to_interim =
        drone->getTurnTime(AngleRad(angle_to_interim - drone->getDirection()));  // TODO min turn time? // turn from current dr.dir to IP
      time_to_interim += drone->getTimeToFlyToInterimPoint(dist_to_interim);

      double dist_to_fire = 0.0;
      double angle_to_fire = 0.0;
      pointmath::trxPointToDistAngle(pos_fire - dropRoute.interm_p, dist_to_fire, angle_to_fire);

      auto turn_between_legs = AngleRad(angle_to_fire - angle_to_interim);

      total_time = time_to_interim;
      total_time += drone->getTurnTime(turn_between_legs);  // TODO min turn time?
      total_time += drone->getTimeToFlyToFP(dist_to_fire);
    }
  }

  if (!dropRoute.has_intermediate_point) {
    double dist{0.0};
    double angle{0.0};
    pointmath::trxPointToDistAngle(pos_fire - start, dist, angle);
    total_time = drone->getTimeToFlyToFP(dist);
    double minTurnTime = drone->getMinTimeToTurn(AngleRad(angle - drone->getDirection()), total_time);
    total_time += minTurnTime;  // turn from current dr.dir to FP
  }

  time_accuracy = total_time - time_total;
  time_total = total_time;
  return total_time;
}

// called at start mission
// solves ballistic task => drop route
// returns false if mission is impossible
auto Mission::calculateMissionDropeRoute(const dto::Target& target) -> bool
{
  /*
  //TODO if needed ???
    constexpr double kMinSpeedRatio = 0.8;
    const double angleToTarget = pointmath::getAngle(target.position - drone->getPosition());
    const double projectedTgtSpeed = currTgt->speed * std::cos(angleToTarget);
   if (drone->attSpeed * kMinSpeedRatio < projectedTgtSpeed) {
      return false;
    } */

  tgt_lead_pos = getTargetLeadPosition(target, timer + ammoFlyTime);  //@sm
  solveDropRoute();                                                   // get ballistic solution
  int count = 0;
  double total_time = calculateTimeForDropRoute(drone->getPosition());  //@cmdr
  double accuracy_s = currTgt->getAccuracyS(kAccuracy_m);
  if (currTgt->speed > kEps) {  // we shall not calculate time accuracy for still target
    // check time accuracy and maybe repeat calculations
    while (std::abs(time_accuracy) > accuracy_s &&
           count < kMaxRecalculations) {  // we are not accurate enough, so we can try to recalculate with new time estimation
      count++;

      tgt_lead_pos = getTargetLeadPosition(target, total_time + ammoFlyTime);
      solveDropRoute();                                              // @ cmdr
      total_time = calculateTimeForDropRoute(drone->getPosition());  //@cmdr
    }

    if (count > countMaxRecalc) {
      countMaxRecalc = count;
    }
  }

  DEBUG("@CMDR@ count=" << count << ' ' << missionStateToStr() << "@ TPos=@" << target.position << "@ Tv=" << currTgt->speed
                        << "@ ttfp=" << total_time);

  return count < kMaxRecalculations;
}

auto Mission::calculateMission() -> bool
{
  const dto::Target& target = currTgt->now;

  if (!calculateMissionDropeRoute(target)) {
    currTgt->state = core::UNREACHABLE;
    breakMission();
    return false;
  }

  state = TO_FIREP;  // initial
  dropPoint = dropRoute.fire_p;

  if (dropRoute.has_intermediate_point) {  // there is interim point, discard mission
    currTgt->state = core::UNREACHABLE;
    breakMission();
    return false;
  }

  // to  TO_FIREP
  destPoint = dropRoute.fire_p;
  destAngle = pointmath::getAngle(destPoint - drone->getPosition());
  timer = time_total - time_to_interim;
  tgt_lead_pos = getTargetLeadPosition(target, timer + ammoFlyTime);  //@snm
  if (recalculateFPOntheRoute(currTgt->now)) {                        //@snm
    currTgt->state = core::UNREACHABLE;
    breakMission();
    return false;
  }

  double ttt = drone->getMinTimeToTurn(destAngle, timer);

  if (ttt > 0.0) {  // we will not be able to complete turn on the move
    //  if (!((!drone->isStopp ed()) || (!drone->isTur ning()))) { TODO
    drone->startDecelerating();  // @snm if not yet turnng or stopped - to be checked inside  //TODO move to drone state
                                 //   }
  }
  else {  // we can turn on the move
    if (destAngle.value != 0.0) {
      double curr_dir = drone->getDirection();
      double turn_thresh = drone->getTurnThreshold();  // TODO move to drone state
      drone->setDroneDirection((destAngle.value < 0.0) ? curr_dir - turn_thresh : curr_dir + turn_thresh);
    }
    // TODO if needed ?? drone->state = drone->speed == drone->attSpeed ? core::MOVING : core::ACCELERATING;
  }

  DEBUG("@SNM " << missionStateToStr() << "@tmr=" << timer << "@MdA" << destAngle << "@MdP@" << destPoint << "@FP@" << dropPoint << "@LP@"
                << tgt_lead_pos);
  return true;
}
}  // namespace core
