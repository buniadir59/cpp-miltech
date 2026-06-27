#pragma once

#include "math/point_math.hpp"
#include "dto/Target.hpp"

#include <cstdint>

namespace core {

enum TgtState : std::uint8_t { ACTIVE, ATTACKED, DESTROYED, UNREACHABLE, UNKNWN };

class TargetControl {  // current information on target available to mission

public:
  TgtState state = UNKNWN;
  double speed;     // tgt sp,  updated with update()
  dto::Target now;  // updated with update()

  pointmath::Point hitCoord;  // coordinate of ammo hit the ground
  double hitTime;             // time ammo hit the ground

  auto update(double tgtTimeStep) -> void; //called every step to obtain new position and speed params 
  
  auto getAccuracyS(double acc_m) -> double;//returns time for which the target makes acc_m distance
  auto getLeadPos(double lead_time_ratio) -> pointmath::Point;
  [[nodiscard]] auto targetStateToStr() const -> const char*
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
