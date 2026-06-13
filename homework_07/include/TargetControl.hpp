#pragma once

#include "math/point_math.hpp"
#include "dto/Target.hpp"

namespace core {

enum TgtState { ACTIVE, ATTACKED, DESTROYED, UNREACHABLE, UNKNWN };

class TargetControl {  // current information on target available to mission

public:
  TgtState state = UNKNWN;
  double speed{};  // tgt sp
  dto::Target now{};

  pointmath::Point hitCoord{};  // coordinate of ammo hit the ground
  double hitTime{};             // time ammo hit the ground

  auto update(double tgtTimeStep) -> void;
  auto getAccuracyS(double acc_m) -> double;

  auto targetStateToStr() const -> const char*
  {
    switch (state) {
      case ACTIVE:
        return "ACTIVE";
      case ATTACKED:
        return "UNDER_ATTACK";
      case DESTROYED:
        return "DESTROYED";
      case UNREACHABLE:
        return "UNREACHABLE";
      default:
        return "UNKNOWN_STATE";
    }
  }

};  // eo TargetState

}  // namespace core
