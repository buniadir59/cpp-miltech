#pragma once

#include "ballistics.hpp"
#include "math/point_math.hpp"
#include "math/angle_math.hpp"
#include "dto/Ammo.hpp"

#include <limits>
#include <cstring>
#include <iostream>

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

struct TargetState {  // current information on target available to drone
  bool has_previous = false;
  double last_known_s = 0.0;
  Point last_known{};
  Point velocity{};  // estimated velocity from two last known positions

  double time_accuracy = 0.0;
  double time_total = 0.0;
  double time_to_interim = 0.0;  // time to reach interim point
  ballistics::DropSolution dropRoute{};

  auto getSpeed() const -> double { return pointmath::getLength(velocity); }

  void update(const Point& new_position, double time_s)
  {  // receive new "real"(interpolated) position from simulation
    if (has_previous) {
      const double dt = time_s - last_known_s;
      if (dt > eps)
        velocity = (new_position - last_known) / dt;
    }
    else {  // velocity stays 0
      has_previous = true;
    }

    last_known = new_position;
    last_known_s = time_s;
  }

  // return projected position for lead targeting
  Point getLeadPosition(double delta_t) const { return last_known + velocity * (delta_t); };

  auto calculateBallisticSolutionAt(double delta_t, ballistics::BallisticsInput& input) -> void
  {
    input.target_pos = getLeadPosition(delta_t);  //@calc BS
    dropRoute = ballistics::compute_drop_solution(input);
  };
};  // eo TargetState #########################

std::ostream& operator<<(std::ostream& os, const drone::TargetState& tgt);

enum MissionState { NONE, TO_INTERIMP, TO_FIREP, FIRED, FAILED, COMPLETED };

struct Mission {
  MissionState state{NONE};
  int tgtTag = -1;
  double timer = 0;       // time left to drop
  double maxSpeed = 0.0;  // max speed to reach interim point

  Point tgt_lead_pos{};
  Point decelerateAtPoint{0, 0};  // point where to start decceleration to reach interim point
  AngleRad destAngle{0.0};        // direction to destination, calculated at start of mission
  Point destPoint{0, 0};

  Point dropPoint{0, 0};  //=dest point when TO_FIREP, initially target from drop_route
  Point aimPoint{0, 0};   // has sense when moving

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

std::ostream& operator<<(std::ostream& os, const drone::Mission& m);

struct SimStep {
  Point pos;              // позиція дрона
  float direction;        // напрямок (рад)
  int state;              // стан автомата (0-4)
  int targetIdx;          // індекс поточної цілі
  Point dropPoint;        // точка скиду (куди летить дрон)
  Point aimPoint;         // куди впаде бомба (якщо скинути зараз)
  Point predictedTarget;  // прогнозована позиція цілі
};

struct DroneConfig {
  Point position{};
  double altitude = 0.0;
  double initial_direction{};
  double attack_speed = 0.0;
  double acceleration_path = 0.0;

  char ammo_name[32] = {};

  double angular_speed = 0.0;
  double turn_threshold = 0.0;

  size_t number_of_targets = 0;
  const dto::Ammo* ammo = nullptr;
};

enum DroneState { STOPPED = 0, ACCELERATING, DECELERATING, TURNING, MOVING };

struct Drone {
  // constant - obtained from input data:
  double alt;       // altitude
  double accPath;   // acceler path
  double attSpeed;  // attack speed

  double angSpeed;   // Кутова швидкість повороту (рад/с)
  double turnThrld;  // Пороговий кут для зупинки (рад)
  size_t nTargets;
  const dto::Ammo* ammo = nullptr;

  // constant - calculated once and saved for future use
  double kAcceleration;      // calculated from acceler time and attack speed
  double kAccuracy_m = 0.0;  // distance to destination to decide it is reached

  // changing during simulation:
  double speed = 0.0;          // current speed, m/s
  DroneState state = STOPPED;  // assume initial state is full stop
  Point coord{};               // initialized from input file
  Point dirXY{};               // direction by X and Y (as Point)  according to dirAngleRad
  AngleRad dirRad{0.0};        // напрямок дрона (радіани, від осі X) //initialized from input file
  Mission mission{};           // current mission

  TargetState* tgts{nullptr};  // known to drone information about targets

  int countMaxRecalc = 0;  // max number of recalculated drop solutions
  int errcode = 0;

  explicit Drone(const DroneConfig& config)
    : alt{config.altitude}
    , accPath{config.acceleration_path}
    , attSpeed{config.attack_speed}
    , angSpeed{config.angular_speed}
    , turnThrld{config.turn_threshold}
    , nTargets(config.number_of_targets)
    , ammo(config.ammo)
    , coord{config.position}
    , dirRad{config.initial_direction}
    , tgts{new TargetState[config.number_of_targets]}
  {
    setDroneDirection(config.initial_direction);
    kAcceleration = attSpeed * attSpeed / (2 * accPath);
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
  auto calculateTimeForDropRoute(Point start, TargetState& tgt) -> double;
  auto recalculateFPOntheRoute(double ammo_f_dist, double time_step) -> int;

  auto getBestTarget() -> int;

  // ## public funcs
  void moveDrone(double dt);
  auto startNewMission(double time_step) -> int;
  auto isOnMission() const -> bool { return mission.tgtTag >= 0; }
  auto breakMission() -> MissionState
  {
    mission.state = NONE;
    mission.tgtTag = -1;
    state = speed > 0.0 ? DECELERATING : STOPPED;
    return NONE;
  }
  auto completeMission() -> void { mission.state = COMPLETED; };
  auto isMissionCompleted() -> bool { return mission.state == COMPLETED; }
  auto continueMission(double time_step) -> bool;  //=> true if fired

  auto getHitCoordAndAmmoFlyTime(Point& hit_pos) -> double;

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

  auto updateSimStep(SimStep& step) const -> void;

  auto freeMemory() -> void
  {
    if (tgts != nullptr) {
      delete[] tgts;
      tgts = nullptr;
    }
  }
};

}  // namespace drone
