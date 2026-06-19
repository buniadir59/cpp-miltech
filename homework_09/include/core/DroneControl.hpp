#pragma once

#include "interfaces/IDroneState.hpp"
#include "math/point_math.hpp"
#include "math/angle_math.hpp"
#include "dto/MissionConfig.hpp"
#include "drone/DroneContext.hpp"

#include <memory>
// #include <cstdint>

namespace core {

// enum DrState: std::uint8_t { STOPPED = 0, ACCELERATING, DECELERATING, TURNING, MOVING }; //TODO

class DroneControl {
  /*   double angSpeed;                // Кутова швидкість повороту (рад/с)
    anglemath::AngleRad destAngle;  // set under mission control */
  drone::DroneContext ctx;
  std::unique_ptr<IDroneState> state;  // TODO //DrState state = STOPPED;  // assume initial state is full stop

public:
  /*   // constant - obtained from input data:
    double alt;        // altitude
    double accPath;    // acceler path
    double attSpeed;   // attack speed
    double turnThrld;  // Пороговий кут для зупинки (рад)
    // constant - calculated once and saved for future use
    double kAcceleration;  // calculated from acceler time and attack speed
    // changing during simulation:
    pointmath::Point coord{};         // initialized from input file
    anglemath::AngleRad dirRad{0.0};  // напрямок дрона (радіани, від осі X) //initialized from input file
    pointmath::Point dirXY{};         // direction by X and Y (as Point)  according to dirAngleRad

    double speed = 0.0;  // current dr speed, m/s */

  /*   auto next = state->execute(ctx);   //TODO
    if (next) {
      state = std::move(next);
    } */

  [[nodiscard]] auto getPosition() const -> const pointmath::Point& { return ctx.coord; };
  [[nodiscard]] auto isMoving() const -> bool; //TODO
  [[nodiscard]] auto isTurning() const -> bool; //TODO
  [[nodiscard]] auto isStopped() const -> bool; //TODO
  [[nodiscard]] auto getDirection() const -> double {return ctx.dirRad.value;};
  [[nodiscard]] auto getSpeed() const -> double {return ctx.speed;};
  [[nodiscard]] auto getTurnThreshold() const -> double {return ctx.turnThrld;};
  [[nodiscard]] auto getStateName() const -> const char* {return state->name();};
 
  auto startAccelerating() -> void; //TODO
  auto startDecelerating() -> void; //TODO
  auto startTurning() -> void; //TODO

  void setDroneDirection(double aR);

  DroneControl(const dto::MissionConfig& config)
    : ctx{config.altitude,
          config.drone_position,
          config.acceleration_path,
          config.attack_speed,
          config.turn_threshold,
          config.angular_speed,
          config.initial_direction}
  {
  }

  auto getTimeToGainAttackSpeed() const -> double;
  auto getMinTimeToTurn(anglemath::AngleRad delta_angle, double time_on_move, double time_step) const -> double;
  auto getTurnTime(anglemath::AngleRad delta) const -> double;
  auto getTimeToFlyToInterimPoint(double dist) const -> double;
  auto getTimeToFlyToFP(double dist_to_fp) const -> double;

  auto move(double dt) -> void;
  auto getAimPoint(double ammoHDist) -> pointmath::Point { return ctx.coord + ctx.dirXY * ammoHDist; };
  auto droneStateToStr() const -> const char*;
};

}  // namespace core