#pragma once

#include "drone/Stopped.hpp"
#include "interfaces/IDroneState.hpp"
#include "math/point_math.hpp"
#include "math/angle_math.hpp"
#include "dto/MissionConfig.hpp"
#include "drone/DroneContext.hpp"

#include <memory>
// #include <cstdint>

namespace core {


class DroneControl {
  drone::DroneContext ctx;
  std::unique_ptr<IDroneState> state; 

public:
  auto goIdle() -> void;             // auto startAccelerating() -> void;
  auto startDecelerating() -> void;  // TODO remove
  auto startTurning() -> void;       // TODO remove

  void setDroneDirection(double aR) { return ctx.setDroneDirection(aR); };  // TODO ?

  [[nodiscard]] auto isTurning() const -> bool;                                     // TODO remove
  [[nodiscard]] auto getSpeed() const -> double { return ctx.speed; };              // TODO remove
  [[nodiscard]] auto getTurnThreshold() const -> double { return ctx.turnThrld; };  // TODO remove

  DroneControl(const dto::MissionConfig& config)
    : ctx{config.altitude,
          config.drone_position,
          config.acceleration_path,
          config.attack_speed,
          config.turn_threshold,
          config.angular_speed,
          config.initial_direction,
          config.time_step}
  {
    state = std::make_unique<drone::Stopped>();  // assume initial state is full stop
  }
  [[nodiscard]] auto getPosition() const -> const pointmath::Point& { return ctx.coord; };  // for M-Proccr -push step
  [[nodiscard]] auto isMoving() const -> bool;                                              // for miss-n fire
  [[nodiscard]] auto getDirection() const -> double { return ctx.dirRad.value; };           // for M-Proccr -push step
  [[nodiscard]] auto getStateName() const -> const char* { return state->name(); };         // for M-Proccr -push step
  [[nodiscard]] auto droneStateToStr() const -> const char*;
  [[nodiscard]] auto getAimPoint(double ammoHDist) -> pointmath::Point { return ctx.coord + ctx.dirXY * ammoHDist; };

  auto setDestinationAngle(double value) -> void { ctx.destAngle = value; };
  auto setDestinationPoint(pointmath::Point dest, bool has_interim_p) -> void;


  auto move() -> void;
  auto getTurnTime(anglemath::AngleRad delta) const -> double { return ctx.getTurnTime(delta); };  // to calculate miss-n time
  auto getMinTimeToTurn(anglemath::AngleRad delta_angle, double time_on_move) const -> double {return ctx.getMinTimeToTurn(delta_angle, time_on_move); };

  // TODO review visibility
  // auto getTimeToGainAttackSpeed() const -> double;

  auto getTimeToFlyToInterimPoint(double dist) const -> double { return ctx.getTimeToFlyToInterimPoint(dist); };  // TODO
  auto getTimeToFlyToFP(double dist_to_fp) const -> double { return ctx.getTimeToFlyToFP(dist_to_fp); };          // TODO
};

}  // namespace core