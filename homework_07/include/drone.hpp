#pragma once

#include "math/point_math.hpp"
#include "math/angle_math.hpp"
#include "dto/Ammo.hpp"
#include "dto/TargetState.hpp"
//#include "dto/DropSolution.hpp"
#include "dto/MissionConfig.hpp"
#include "dto/Mission.hpp"
//#include "providers/JsonTargetProvider.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "dto/SimStep.hpp"

#include <limits>
#include <cstring>
//#include <iostream>

using Point = pointmath::Point;
using AngleRad = anglemath::AngleRad;

/* **** drone.hpp / drone.cpp
  TargetState::
    predicts target position for use in lead targeting

  Mission::
    steers drone along drop path

  DroneConfig:: for initial data from input file
  Drone::
    owns TargetState* tgts
    borrows const ammo::Ammo* ammo
    updates drone params
    chooses target
  */

namespace drone {

constexpr double eps = std::numeric_limits<double>::epsilon();

enum DroneState { STOPPED = 0, ACCELERATING, DECELERATING, TURNING, MOVING };

struct Drone {
  // constant - obtained from input data:
  double alt;       // altitude
  double accPath;   // acceler path
  double attSpeed;  // attack speed

  double angSpeed;   // Кутова швидкість повороту (рад/с)
  double turnThrld;  // Пороговий кут для зупинки (рад)
  size_t nTargets = 5; //TODO !!!
  const dto::Ammo* ammo = nullptr;//const dto::Ammo* ammo = nullptr;

  // constant - calculated once and saved for future use
  double kAcceleration;      // calculated from acceler time and attack speed
  double kAccuracy_m = 0.0;  // distance to destination to decide it is reached

  // changing during simulation:
  double speed = 0.0;          // current speed, m/s
  DroneState state = STOPPED;  // assume initial state is full stop
  Point coord{};               // initialized from input file
  Point dirXY{};               // direction by X and Y (as Point)  according to dirAngleRad
  AngleRad dirRad{0.0};        // напрямок дрона (радіани, від осі X) //initialized from input file
  dto::Mission mission{};           // current mission

  dto::TargetState* tgts{nullptr};  // known to drone information about targets

  int countMaxRecalc = 0;  // max number of recalculated drop solutions
  int errcode = 0;

  ITargetProvider* tgtProvider = nullptr;

  explicit Drone(const dto::MissionConfig& config,  ITargetProvider* tgtProvider)
    : alt{config.altitude}
    , accPath{config.acceleration_path}
    , attSpeed{config.attack_speed}
    , angSpeed{config.angular_speed}
    , turnThrld{config.turn_threshold}
   // , nTargets(config.)
    , coord{config.drone_position}
    , dirRad{config.initial_direction}
    , tgts{new dto::TargetState[config.maxTargets]}
  {
    setDroneDirection(config.initial_direction);
    kAcceleration = attSpeed * attSpeed / (2 * accPath);  
    
    this->tgtProvider = tgtProvider;
  }

  void setDroneDirection(double aR)
  {
    dirRad = aR;
    dirXY = pointmath::cossin(dirRad.value);
  };

  auto getTimeToGainAttackSpeed() const -> double { return (state == MOVING) ? 0.0 : (attSpeed - speed) / kAcceleration; }

  auto getAmmoFlyDist() const -> double;
  auto getAmmoFlyTime() const -> double;

  auto getMinTimeToTurn(double abs_delta_angle, double time_on_move, double time_step);
  auto getTurnTime(AngleRad delta) const -> double;
  auto getTimeToFlyToInterimPoint(double dist) const -> double;

  auto getTimeToFlyToFP(double dist_to_fp) const -> double;
  auto calculateTimeForDropRoute(Point start, dto::TargetState& tgt) -> double;
  auto recalculateFPOntheRoute(double ammo_f_dist, double time_step) -> int;

  auto getBestTarget() -> int;

  // ## public funcs
  void moveDrone(double dt);
  auto startNewMission(double time_step) -> int;
  auto isOnMission() const -> bool { return mission.tgtTag >= 0; }
  auto breakMission() -> dto::MissionState
  {
    mission.state = dto::NONE;
    mission.tgtTag = -1;
    state = speed > 0.0 ? DECELERATING : STOPPED;
    return dto::NONE;
  }
  auto completeMission() -> void { mission.state = dto::COMPLETED; };
  auto isMissionCompleted() -> bool { return mission.state == dto::COMPLETED; }
  auto continueMission(double time_step) -> bool;  //=> true if fired

  auto getHitCoordAndAmmoFlyTime(Point& hit_pos) -> double;

//TODO
  Point getLeadPosition(size_t tgtIdx, double delta_t) const; // { return last_known + velocity * (delta_t); };
  auto droneStateToStr() const -> const char*
  {
    switch (state) {
      case STOPPED:
        return "STOPPED";
      case ACCELERATING:
        return "ACCELERATING";
      case DECELERATING:
        return "DECELERATING";
      case TURNING:
        return "TURNING";
      case MOVING:
        return "MOVING";
      default:
        return "UNKNOWN_STATE";
    }
  }

  auto updateSimStep(dto::SimStep& step) const -> void;

  auto freeMemory() -> void
  {
    if (tgts != nullptr) {
      delete[] tgts;
      tgts = nullptr;
    }
  }
};

}  // namespace drone
