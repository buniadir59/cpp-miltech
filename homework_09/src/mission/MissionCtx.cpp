#include "mission/MissionCtx.hpp"
#include "core/TargetControl.hpp"
#include "core/DroneControl.hpp"
#include "math/point_math.hpp"
#include "math/angle_math.hpp"
#include "interfaces/ISimulationClock.hpp"
#include "config/defines.hpp"

#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace {

constexpr double kEps = 1e-9;
constexpr int kMaxRecalculations = 6;  // for drop route
constexpr double kMinSpeedRatio = 0.8;

}  // namespace

namespace mission {

// checks what if we fire now()
// returns true if fired
auto MissionCtx::_checkFireCondition() -> bool
{
  pointmath::Point tlp = _getCurrTgtLeadPos(drone->getInstantAmmoFFTime());  //@cfcond
  pointmath::Point aim_p = drone->getInstantAimPoint();
  double hit_dist = pointmath::getLength(aim_p - tlp);
  if (hit_dist <= kAccuracy_m) {  // Fire
    firePoint = drone->getPosition();
    currTgt->state = core::ATTACKED;
    currTgt->hitCoord = drone->getInstantAimPoint();
    currTgt->hitTime = simClock->nowS() + drone->getInstantAmmoFFTime();
    LOG("H=>" << currTgt->hitTime << " _ _ _ _ _ _ _ Fired! T#" << currentTgtTag << " hitXY " << currTgt->hitCoord << " _ _ _ _ _ hitXY "
              << currTgt->hitCoord);

    return true;
  }
  return false;
}

// returns -1 if fired, 0 if direct manuver to FP is to continue, or error code
// for current tgt based on current drone params
auto MissionCtx::calcAttackRoute() -> int
{
  if (_checkFireCondition()) {
    return -1;
  }

  double time_acc = currTgt->getAccuracyS(kAccuracy_m);
  double dist2AttSp = drone->getDistToGainAttSpeed();
  double time2AttSp = drone->getTimeToGainAttSpeed();
  pointmath::Point drPos = drone->getPosition();
  double baseFFtime = drone->getBaseAmmoFFTime();
  double baseFFdist = drone->getBaseAmmoFFDist();
  double drDir = drone->getDirection();

  double angle_to_tgt;
  double tmr = time2AttSp;  // initial minimal estimation for time to FP -> time to gain Att speed
  int count = 0;
  int res_code = 3;  // =>too many recalculations

  while (++count < kMaxRecalculations) {
    // get tgt lead pos, angle and dist  to tgt
    tgtLeadPos = _getCurrTgtLeadPos(tmr + baseFFtime);  //@cfp saved in context
    double dist_to_tgt;
    pointmath::trxPointToDistAngle(tgtLeadPos - drPos, dist_to_tgt, angle_to_tgt);

    firePoint = tgtLeadPos - pointmath::cossin(angle_to_tgt) * baseFFdist;  // just to report
    if (dist_to_tgt < baseFFdist + dist2AttSp) {                            // we aren't able to gain att speed TODO ?? week condition?
      res_code = 1;
      break;
    }
    double dist2fp;
    double time2fp;
    // get dist to FP and angle to turn on the way
    dist2fp = dist_to_tgt - baseFFdist;
    time2fp = time2AttSp + (dist2fp - dist2AttSp) / drone->getAttSpeed();  // acc + cruize time, turn not accounted

    double min_time_to_turn = drone->getMinTimeToTurn(angle_to_tgt - drDir, time2fp);

    if (min_time_to_turn > kEps) {
      res_code = 2;  // break mission => too much turn needed
      break;
    }

    double timeFPAccuracy = time2fp - tmr;
    tmr = time2fp;
    if (std::fabs(timeFPAccuracy) < time_acc) {
      res_code = 0;
      break;  // accuracy is ok
    }
  }  // eo while

  if (!res_code) {  // ok, update destination for drone
    drone->setDestToAttack(firePoint, angle_to_tgt);
    return 0;
  }

  currTgt->state = core::UNREACHABLE;
  return res_code;
}

auto MissionCtx::breakMission() -> void
{
  drone->flyAway();
  currentTgtTag = -1;
};

// returns position of current target at lead_time sec from now based on current position and velocity info
auto MissionCtx::_getCurrTgtLeadPos(double lead_time) const -> pointmath::Point
{
  if (currTgt == nullptr) {
    throw std::runtime_error("!!! no target");
  }
  return currTgt->getLeadPos(lead_time / tgtTimeStep);
}

// returns index of the active target starting from the ind,
// or -1 if no such target
auto MissionCtx::_getNextTarget(int idx) const -> int
{
  int start_idx = idx < 0 ? 0 : idx;
  auto it = std::find_if(tgts.begin() + start_idx, tgts.end(), [](const core::TargetControl& tgt) { return tgt.state == core::ACTIVE; });

  return it != tgts.end() ? std::distance(tgts.begin(), it) : -1;
}

auto MissionCtx::getNextTarget() -> int
{
  int newIdx;  
  while ((newIdx = _getNextTarget(0)) >= 0) {
    // check  if worth to pursue - check if we have a speed gain over the target
    double angle2T = pointmath::getAngle(tgts[newIdx].now.position - drone->getPosition());
    if (drone->getAttSpeed() * kMinSpeedRatio < tgts[newIdx].speed * std::cos(angle2T)) {  // not reachable
      tgts[newIdx].state = core::UNREACHABLE;
    }
    else {
      break;  // ok to proceed
    };
  }
  return newIdx;
}

}  // namespace mission