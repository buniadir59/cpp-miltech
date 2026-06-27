#pragma once

#include "drone/Stopped.hpp"
#include "math/point_math.hpp"
#include "math/angle_math.hpp"
#include "dto/MissionConfig.hpp"
#include "drone/DroneContext.hpp"

#include <memory>

namespace dto {
struct Ammo;
}

class IDroneState;

namespace core {

class DroneControl {
  drone::DroneContext ctx;
  std::unique_ptr<IDroneState> state;

public:
  auto setDestToAttack(pointmath::Point dest, double angle) -> void  // for calc route
  {
    ctx.setDestToFP(dest);
    ctx.angleToTLpos = angle;
  };

  auto flyAway() -> void  // for break Mission
  {
    ctx.setDestToFP(ctx.coord + ctx.dirXY * ctx.accPath);
    ctx.angleToTLpos = ctx.dirRad;
  };

  auto execFly() -> void;  // fly following destination params

  auto setSolver(IBallisticSolver* s) -> bool
  {
    if (s == nullptr) {
      return false;
    }

    ctx.solver = s;
    return ctx.updateBasicAmmoRes();
  };

  auto setAmmo(const dto::Ammo* a) -> bool
  {
    if (a == nullptr) {
      return false;
    }

    ctx.ammo = a;
    return ctx.updateBasicAmmoRes();
  };

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

  // below are for M-Proccr -push step
  [[nodiscard]] auto getPosition() const -> const pointmath::Point& { return ctx.coord; };  // for M-Proccr -push step
  [[nodiscard]] auto getDirection() const -> double { return ctx.dirRad.value; };           // for M-Proccr -push step
  [[nodiscard]] auto getStateName() const -> const char* { return state->name(); };
  [[nodiscard]] auto getInstantAimPoint() -> pointmath::Point { return ctx.coord + ctx.dirXY * ctx.ballResult.hDist; };

  // below are to calculate attack route
  [[nodiscard]] auto getAttSpeed() const -> double { return ctx.attSpeed; }
  [[nodiscard]] auto getTimeToGainAttSpeed() const -> double { return ctx.timeToGainAttSpeed; };
  [[nodiscard]] auto getDistToGainAttSpeed() const -> double { return ctx.distToGainAttSpeed; };
  [[nodiscard]] auto getBaseAmmoFFTime() -> double { return ctx.ballResult.ffTime; };
  [[nodiscard]] auto getBaseAmmoFFDist() -> double { return ctx.ballResult.hDist; };
  [[nodiscard]] auto getMinTimeToTurn(anglemath::AngleRad delta_angle, double time_on_move) const -> double
  {
    return ctx.getMinTimeToTurn(delta_angle, time_on_move);
  };

  [[nodiscard]] auto getInstantAmmoFFTime() -> double { return ctx.ammoBaseFFTime; };  // to estimate fire condition

};

}  // namespace core