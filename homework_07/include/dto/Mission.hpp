#pragma once

#include "math/point_math.hpp"
#include "math/angle_math.hpp"
//#include "dto/DropSolution.hpp"

namespace dto {
enum MissionState { NONE, TO_INTERIMP, TO_FIREP, FIRED, FAILED, COMPLETED };
 
struct Mission {
  MissionState state{NONE};
  int tgtTag = -1;
  double timer = 0;       // time left to drop
  double maxSpeed = 0.0;  // max speed to reach interim point

  pointmath::Point tgt_lead_pos{};
  pointmath::Point decelerateAtPoint{0, 0};  // point where to start decceleration to reach interim point
  anglemath::AngleRad destAngle{0.0};        // direction to destination, calculated at start of mission
  pointmath::Point destPoint{0, 0};

  pointmath::Point dropPoint{0, 0};  //=dest point when TO_FIREP, initially target from drop_route
  pointmath::Point aimPoint{0, 0};   // has sense when moving

    
 auto missionStateToStr() const -> const char*
  {
    switch (state) {
      case NONE:
        return "NONE";
      case TO_INTERIMP:
        return "TO_INTERIMP";
      case TO_FIREP:
        return "TO_FIREP";
      case FIRED:
        return "FIRED";
      case FAILED:
        return "FAILED";
      case COMPLETED:
        return "COMPLETED";
      default:
        return "UNKNOWN_STATE";
    }
  }
};  // eo Mission ############################

//std::ostream& operator<<(std::ostream& os, const dto::Mission& m);
}