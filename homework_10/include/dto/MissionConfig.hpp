#pragma once

#include "math/point_math.hpp"

#include <cstddef>

namespace dto {

struct MissionConfig {
  // drone
  double kAccelerationPath;
  double kAltitude;
  double attackSpeed;
  double maxAngularSpeedRadPerS;
  double initialDirection;
  pointmath::Point initialPosition;
  double turnThreshold;
  // simulation
  double hitRadius;
  double timeStep;
  double targetTimeStep;
  double physicsTimeStep;
  double targetArrayTimeStep;
  double timeScale;

  size_t nAmmos = 0;
};

}  // namespace dto