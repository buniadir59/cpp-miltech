#pragma once

#include "core_/TargetControl.hpp"
#include "interfaces/IMissionState.hpp"
#include "dto/DroneInterfaceStructures.hpp"
#include "dto/MissionConfig.hpp"
#include "math/point_math.hpp"
#include "math/angle_math.hpp"
#include "config/defines.hpp"

#include <memory>
#include <vector>

namespace mission {

class Idle final : public IMissionState {
public:
  std::unique_ptr<IMissionState> execute(MissionCtx& ctx) override;
  const char* name() const override { return "mIdle"; };
};

class Attack final : public IMissionState {
public:
  std::unique_ptr<IMissionState> execute(MissionCtx& ctx) override;
  const char* name() const override { return "mAtt"; };
};

struct MissionCtx {
  const dto::MissionConfig& mconf;
  std::vector<core::TargetControl>& tgts;
  dto::DroneTelemetry telemetry;

  const double kAccuracy_m;  // distance to destination to decide it is reached
  const double kAccTime;
  const double kAcceleration;

  double timeToGainAttSpeed;
  double distToGainAttSpeed;
  double timeToStop;
  double distToStop;
  double instantAmmoFFTime;
  double instantAmmoFFDist;
  double ammoBaseFFTime;
  double ammoBaseHDist;

  int currentTgtTag = -1;                  // no target
  core::TargetControl* currTgt = nullptr;  // changes state of the target if cannot be reached

  pointmath::Point instantAimPoint;
  pointmath::Point tgtLeadPos;  //  up-dated @ calculating route to FP, used in json steps
  pointmath::Point firePoint;   //  up-dated @ calculating route to FP, used in json steps
  double angle_to_tgt;          //  up-dated @ calculating route to FP
  double time2fp;
  double dist2fp;
  double hit_dist;
  int res_code;
  bool fired{false};
  dto::DroneCommand cmd;

  auto breakMission() -> void
  {
    currentTgtTag = -1;
    cmd = {dto::MOVING, 0.0};
  }

  [[nodiscard]] auto _getNextTarget(int idx) const -> int;
  [[nodiscard]] auto getNextTarget() -> int;
  auto setCurrentTgtTag(int tag) -> void
  {
    if (tag >= 0) {
      currentTgtTag = tag;
      currTgt = &tgts[tag];
    }
  };

  [[nodiscard]] auto _getCurrTgtLeadPos(double lead_time) const -> pointmath::Point;
  [[nodiscard]] auto calcAttackRoute() -> int;
  [[nodiscard]] auto _checkFireCondition() -> bool;
  [[nodiscard]] auto getDroneCoord() const -> pointmath::Point
  {
    return {static_cast<double>(telemetry.x), static_cast<double>(telemetry.y)};
  }
  [[nodiscard]] auto getMinTimeToTurn(anglemath::AngleRad delta_angle, double time_on_move) const -> double;
  auto _updateSpeedDependentCtx() -> void;

  MissionCtx(const dto::MissionConfig& mconf, std::vector<core::TargetControl>& tgts)
    : mconf(mconf)
    , tgts(tgts)
    , kAccuracy_m(mconf.timeStep * mconf.attackSpeed * ACCURACY_COEFF)
    , kAccTime(mconf.kAccelerationPath * 2.0 / mconf.attackSpeed)
    , kAcceleration(mconf.attackSpeed * mconf.attackSpeed / (mconf.kAccelerationPath * 2.0))
  {
    timeToGainAttSpeed = kAccTime;
    distToGainAttSpeed = mconf.kAccelerationPath;
    timeToStop = 0.0;
    distToStop = 0.0;
  };
};

}  // namespace mission