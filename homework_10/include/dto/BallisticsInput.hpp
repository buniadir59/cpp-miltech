#pragma once

#include "dto/Ammo.hpp"

namespace dto {

struct BallisticsInput {  // only const for simulation params
  double drone_z = 0.0;
  double attack_speed = 0.0;
  double acceleration_path = 0.0;

  // ammo
  double mass = 0.0;
  double drag = 0.0;
  double lift = 0.0;

  BallisticsInput& setAmmoParams(const dto::Ammo& ammo)
  {
    mass = ammo.mass;
    drag = ammo.drag;
    lift = ammo.lift;
    return *this;
  }
  BallisticsInput& setDroneAltitude(double alt)
  {
    drone_z = alt;
    return *this;
  }
  BallisticsInput& setDroneAccelerationPath(double acc_path)
  {
    acceleration_path = acc_path;
    return *this;
  }
  BallisticsInput& setDroneAttackSpeed(double att_speed)
  {
    attack_speed = att_speed;
    return *this;
  }
};

}  // namespace dto
