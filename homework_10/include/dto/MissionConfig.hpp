#pragma once

#include "math/point_math.hpp"

#include <cstddef>

namespace dto {

struct MissionConfig {
  // drone
  double accelerationPath;
  double altitude;
  double attackSpeed;
  double angularSpeed;
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

  //  const char* targetsPath; //TODO
};

}  // namespace dto