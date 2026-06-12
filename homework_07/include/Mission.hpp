#pragma once

#include "DroneControl.hpp"
#include "dto/Target.hpp"
#include "TargetControl.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "math/point_math.hpp"
#include "math/angle_math.hpp"


namespace core { //namespace dto {

enum MissionState { NONE, TO_INTERIMP, TO_FIREP};
 
class Mission {

  IBallisticSolver* solver;    

  dto::DropSolution dropRoute{};  

  TargetControl* currTgt = nullptr;

  double time_step = 0;
  double time_accuracy = 0.0;
  double time_total = 0.0;
  double time_to_interim = 0.0;  // time to reach interim point
 // double accuracy_s{};
  int countMaxRecalc = 0;  // max number of recalculated drop solutions
  int missionResultCode = 0;
  double kAccuracy_m = 0.0;  // distance to destination to decide it is reached
  double maxSpeed = 0.0;  // drone max speed to reach interim point
  pointmath::Point decelerateAtPoint{0, 0};  // point where to start decceleration to reach interim point
  
  auto calculateMission() -> bool;

  auto recalculateFPOntheRoute(dto::Target& target) -> int;
  auto recalculateTimeToFP(dto::Target& target) -> double;
  auto calculateTimeForDropRoute(const pointmath::Point& start) -> double;
  auto calcTimeToFP() -> double; //cosine theorema
  auto calculateMissionDropeRoute(const dto::Target&  target) -> bool;
  
  auto solveDropRoute() -> void;
  auto getTargetLeadPosition(const dto::Target& tgt, double deltaT)const -> pointmath::Point  {return tgt.position + tgt.velocity * deltaT;}

public:
  DroneControl* drone = nullptr;  
  MissionState state{NONE};

  double ammoHorizDist = 0.0;
  double ammoFlyTime = 0.0;
  double timer = 0;       // time left to drop

  anglemath::AngleRad destAngle{0.0};        // direction to destination, calculated at start of mission
  pointmath::Point destPoint{0, 0};
  pointmath::Point tgt_lead_pos{};
  pointmath::Point dropPoint{0, 0};  //=dest point when TO_FIREP, initially target from drop_route
  
  auto init(double time_step, DroneControl* drone, const dto::Ammo& ammo) -> void;

  auto isOnMission() const -> bool { return state != core::NONE; };
  auto startNewMission(TargetControl& tgt) -> bool; 
  auto continueMission() -> bool;
  auto breakMission() -> void { state = core::NONE; }; 

  auto setSolver(IBallisticSolver* s)-> void;

  Mission(IBallisticSolver* solver) :
    solver(solver)
  {
  }

  auto missionStateToStr() const -> const char*
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
};  // eo Mission ############################


}