#pragma once

#include "core/TargetControl.hpp"
#include "math/point_math.hpp"

#include <vector>

class ISimulationClock;

namespace core {
class DroneControl;
}  // namespace core

namespace mission {

struct MissionCtx {
  const ISimulationClock* simClock{nullptr};  // const - only reads time
  std::vector<core::TargetControl>& tgts;
  core::DroneControl* drone = nullptr;

  double tgtTimeStep{0.0};  // from m config at init time
  double kAccuracy_m{0.0};  // distance to destination to decide it is reached


  int currentTgtTag = -1;                  // no target
  core::TargetControl* currTgt = nullptr;  // changes state of the target if cannot be reached

  pointmath::Point tgtLeadPos;  //  up-dated @ calculating route to FP, used in json steps
  pointmath::Point firePoint;   //  up-dated @ calculating route to FP, used in json steps

  auto getNextTarget() -> int;  // calls _getNextTarget, repeats search from the begining if needed
  auto setCurrentTgtTag(int tag) -> void
  {
    if (tag >= 0) {
      currentTgtTag = tag;
      currTgt = &tgts[tag];
    }
  };
  auto calcAttackRoute() -> int;

  auto _checkFireCondition() -> bool;
  auto _getCurrTgtLeadPos(double lead_time) const -> pointmath::Point;
  auto breakMission() -> void;
  // returns index of the active target starting from the idx till end of vector
  // or-1 if no such target
  auto _getNextTarget(int idx) const -> int;

  MissionCtx(std::vector<core::TargetControl>& tgts, ISimulationClock* clock)
    : simClock(clock)
    , tgts(tgts){};
};

}  // namespace mission