#include "mission/MissionCtx.hpp"
#include "config/TimeTracker.hpp"
#include "dto/DroneInterfaceStructures.hpp"
#include "math/point_math.hpp"
#include "math/angle_math.hpp"

#define DBG_MODE
#ifdef DBG_MODE                // #endif
#include "config/defines.hpp"  //for DEBUG
#endif

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
  pointmath::Point tlp = _getCurrTgtLeadPos(instantAmmoFFTime);  //@cfcond
  hit_dist = pointmath::getLength(instantAimPoint - tlp);
  if (hit_dist <= kAccuracy_m) {  // Fire
    firePoint = {telemetry.x, telemetry.y};
    currTgt->state = core::ATTACKED;
    currTgt->hitCoord = instantAimPoint;
    auto tnow = TimeTracker::getInstance().getElapsed();
    currTgt->hitTime = tnow + instantAmmoFFTime;
    LOG(tnow << "  T#" << currentTgtTag << " Fired=>" << currTgt->hitTime << " hitXY: " << currTgt->hitCoord.x << ' ' << currTgt->hitCoord.y
             << " TLP: " << tlp.x << " " << tlp.y << " h_d=" << hit_dist << " acc=" << kAccuracy_m << " %"
             << std::fmod(tnow, mconf.targetArrayTimeStep));
    fired = true;
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
  pointmath::Point drPos = getDroneCoord();
  double drDir = static_cast<double>(telemetry.dir);

  double tmr = timeToGainAttSpeed;  // initial minimal estimation for time to FP -> time to gain Att speed
  int count = 0;
  res_code = 3;  // =>too many recalculations

  while (++count < kMaxRecalculations) {
    // get tgt lead pos, angle and dist  to tgt
    tgtLeadPos = _getCurrTgtLeadPos(tmr + ammoBaseFFTime);  //@cfp saved in context
    double dist_to_tgt;
    pointmath::trxPointToDistAngle(tgtLeadPos - drPos, dist_to_tgt, angle_to_tgt);

    firePoint = tgtLeadPos - pointmath::cossin(angle_to_tgt) * ammoBaseHDist;  // just to report
    if (dist_to_tgt < ammoBaseHDist + distToGainAttSpeed) {                    // we aren't able to gain att speed
    // TODO to improve ->check if we are able to decelerate and turn
      res_code = 1;
#ifdef DBG_MODE  // #endif
      DEBUG(TimeTracker::getInstance().getElapsed()
            << " 1-T# " << currentTgtTag  // resC=1
            << " d2tgt " << dist_to_tgt << " aBHD " << ammoBaseHDist << " d2AS " << distToGainAttSpeed << " dist2tgt<aBHD+d2AS" << " a2T "
            << anglemath::rad2Grad(angle_to_tgt) << " drD " << anglemath::rad2Grad(drDir));
#endif
      break;
    }
    
    //  get dist to FP and angle to turn on the way
    dist2fp = dist_to_tgt - ammoBaseHDist;
    time2fp = timeToGainAttSpeed + (dist2fp - distToGainAttSpeed) / mconf.attackSpeed;  // acc + cruize time, turn not accounted

    double min_time_to_turn = getMinTimeToTurn(angle_to_tgt - drDir, time2fp);

    if (min_time_to_turn >= mconf.timeStep) {  // kEps
      res_code = 2;                            // break mission => too much turn needed
#ifdef DBG_MODE                                // #endif
      DEBUG(TimeTracker::getInstance().getElapsed()
            << " 2-T# "  // resC=2
            << currentTgtTag << " mt2t= " << min_time_to_turn << " dist2fp " << dist2fp << " time2fp " << time2fp << " d2AS "
            << distToGainAttSpeed << " t2AS " << timeToGainAttSpeed << "tlp " << tgtLeadPos.x << "" << tgtLeadPos.y << " a2T "
            << anglemath::rad2Grad(angle_to_tgt) << " drD " << anglemath::rad2Grad(drDir));
#endif
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
    double delta_angle = anglemath::normalizeAngle(angle_to_tgt - drDir);
    double ang_speed = delta_angle / mconf.timeStep;
    cmd.state = dto::MOVING;
    cmd.angleSpeed = static_cast<float>(ang_speed);
    if (std::fabs(ang_speed) > mconf.maxAngularSpeedRadPerS) {  // speed will be limited by drone physics
      double av_ang_speed = std::fabs(delta_angle / time2fp);
      if (av_ang_speed > mconf.maxAngularSpeedRadPerS) {
        if (telemetry.speed == 0.0) {
          cmd.state = dto::TURNING;
        }
        else {
          cmd.state = dto::DECELERATING;
        }
      }
    }
    return 0;
  }  // eo ok
#ifdef DBG_MODE  // #endif

  else {
    if (res_code == 3) {
      DEBUG(TimeTracker::getInstance().getElapsed()
            << " 3-T# "  // resC=3
            << currentTgtTag << " tAcc " << time2fp - tmr << " tmr " << tmr << " resC=3 dist2fp " << dist2fp << " time2fp " << time2fp
            << " d2AS " << distToGainAttSpeed << " t2AS" << timeToGainAttSpeed);
    }
  }
#endif
  currTgt->state = core::UNREACHABLE;
  return res_code;
}

auto MissionCtx::_updateSpeedDependentCtx() -> void
{
  pointmath::Point dirXY = pointmath::cossin(telemetry.dir);
  instantAimPoint = getDroneCoord() + dirXY * instantAmmoFFDist;

  if (telemetry.speed == static_cast<float>(mconf.attackSpeed)) {
    timeToGainAttSpeed = 0.0;
    distToGainAttSpeed = 0.0;
    timeToStop = kAccTime;
    distToStop = mconf.kAccelerationPath;
  }
  else if (telemetry.speed < kEps) {
    timeToGainAttSpeed = kAccTime;
    distToGainAttSpeed = mconf.kAccelerationPath;
    timeToStop = 0.0;
    distToStop = 0.0;
  }
  else {
    timeToStop = telemetry.speed / kAcceleration;
    timeToGainAttSpeed = kAccTime - timeToStop;
    distToStop = kAcceleration * timeToStop * timeToStop / 2.0;
    distToGainAttSpeed = mconf.kAccelerationPath - distToStop;
  }
}

auto MissionCtx::getMinTimeToTurn(anglemath::AngleRad delta_angle, double time_on_move) const -> double
{
  double abs_delta_angle = std::fabs(delta_angle.value);
  double turn_on_the_move = mconf.maxAngularSpeedRadPerS * time_on_move;
  double delta = abs_delta_angle - turn_on_the_move;
  return delta > kEps ? delta / mconf.maxAngularSpeedRadPerS : 0.0;
};

// returns position of current target at lead_time sec from now based on current position and velocity info
auto MissionCtx::_getCurrTgtLeadPos(double lead_time) const -> pointmath::Point
{
  if (currTgt == nullptr) {
    throw std::runtime_error("!!! no target");
  }
  return currTgt->getLeadPos(lead_time);
}

// returns index of the active target starting from the ind,
// or -1 if no such target
auto MissionCtx::_getNextTarget(int idx) const -> int  // TODO lambda
{
  int start_idx = idx < 0 ? 0 : idx;
  auto it = std::find_if(tgts.begin() + start_idx, tgts.end(), [](const core::TargetControl& tgt) { return tgt.state == core::ACTIVE; });

  return it != tgts.end() ? std::distance(tgts.begin(), it) : -1;
}

// calls _getNextTarget, repeats search from the begining if needed
auto MissionCtx::getNextTarget() -> int
{
  int newIdx;
  while ((newIdx = _getNextTarget(0)) >= 0) {
    // check  if worth to pursue - check if we have a speed gain over the target
    double angle2T = pointmath::getAngle(tgts[newIdx].now.position - getDroneCoord());
    if (mconf.attackSpeed * kMinSpeedRatio < tgts[newIdx].speed * std::cos(angle2T)) {  // not reachable
      tgts[newIdx].state = core::UNREACHABLE;
    }
    else {
      break;  // ok to proceed
    };
  }
  return newIdx;
}

// finds new target and check if it is reachable
// if found returns ptr to  respective state
// if not found returns nullptr
std::unique_ptr<IMissionState> Idle::execute(MissionCtx& ctx)
{
  int tag;
  while ((tag = ctx.getNextTarget()) >= 0) {
    // try to start new mission
    ctx.setCurrentTgtTag(tag);
    if (!ctx.calcAttackRoute()) {  // ok, we are attacking next target
      return std::make_unique<mission::Attack>();
    }
  }

  ctx.breakMission();
  return nullptr;  // continue idle, no need to check other states as we already know that has Next is true
}

// finds new target and check if it is reachable
// if found returns ptr to  respective state
// if not found returns nullptr
std::unique_ptr<IMissionState> Attack::execute(MissionCtx& ctx)
{
  // re-calculate route to FP ()
  int res = ctx.calcAttackRoute();

  if (!res) {  // continue to FP
    return nullptr;
  }

  int tag;
  while ((tag = ctx.getNextTarget()) >= 0) {
    // try to start new mission
    ctx.setCurrentTgtTag(tag);
    res = ctx.calcAttackRoute();
    if (!res) {  // ok, we are attacking next target
      return nullptr;
    }
  }

  ctx.breakMission();
  return std::make_unique<mission::Idle>();
}

}  // namespace mission