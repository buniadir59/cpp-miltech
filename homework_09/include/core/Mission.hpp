#pragma once

#include "interfaces/IBallisticSolver.hpp"
#include "dto/DropSolution.hpp"
#include "math/point_math.hpp"
#include "math/angle_math.hpp"

#include <cstdint>

namespace dto {
struct Target;
struct MissionConfig;
}

namespace core {
class TargetControl;
class DroneControl;

enum MissionState : std::uint8_t { NONE, TO_INTERIMP, TO_FIREP };

class Mission {
  IBallisticSolver* solver;

  dto::DropSolution dropRoute{};

  TargetControl* currTgt = nullptr;

  double tgtTimeStep{0.0};
  double time_step{0.0}; //for timer
  double time_accuracy{0.0};
  double time_total{0.0};
  double time_to_interim{0.0};  // time to reach interim point

  int countMaxRecalc = 0;  // max number of recalculated drop solutions
  int missionResultCode = 0;
  double kAccuracy_m{0.0};  // distance to destination to decide it is reached

  MissionState state{NONE};

  anglemath::AngleRad destAngle;  // direction to destination, calculated at start of mission
  pointmath::Point destPoint;

  DroneControl* drone = nullptr;

  auto calculateMission() -> bool;

  auto recalculateFPOntheRoute(dto::Target& target) -> int;
  auto recalculateTimeToFP(dto::Target& target) -> double;
  auto calculateTimeForDropRoute(const pointmath::Point& start) -> double;
  auto calculateMissionDropeRoute(const dto::Target& target) -> bool;

  auto solveDropRoute() -> void;
  [[nodiscard]] auto getTargetLeadPosition(const dto::Target& tgt, double deltaT) const -> pointmath::Point;

public:
  double ammoHorizDist = 0.0;
  double ammoFlyTime = 0.0;
  double timer = 0;  // time left to drop
  pointmath::Point tgt_lead_pos;
  pointmath::Point dropPoint;  //=dest point when TO_FIREP, initially target from drop_route

  auto init(const dto::MissionConfig* mconf,  DroneControl* drone_ptr, const dto::Ammo* ammo) -> void;

  [[nodiscard]] auto isOnMission() const -> bool { return state != core::NONE; };
  auto startNewMission(TargetControl& tgt) -> bool;
  auto continueMission() -> bool;
  auto breakMission() -> void { state = core::NONE; };

  auto setSolver(IBallisticSolver* s) -> void;

  Mission(IBallisticSolver* solver)
    : solver(solver)
  {
  }

  [[nodiscard]] auto missionStateToStr() const -> const char*
  {
    switch (state) {
      case NONE:
        return "_IDLE";
      case TO_INTERIMP:
        return "TO_INTERIMP";
      case TO_FIREP:
        return "TO_FIREP";
      default:
        return "_UNKNOWN";
    }
  }
};  // eo Mission 

}  // namespace core