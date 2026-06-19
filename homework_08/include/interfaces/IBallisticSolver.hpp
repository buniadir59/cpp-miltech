#pragma once

#include "dto/DropSolution.hpp"
#include "dto/Ammo.hpp"
#include "math/point_math.hpp"


class IBallisticSolver {
public:
  virtual auto solve(const pointmath::Point& drone_position,
                     const pointmath::Point& target_position,
                     double altitude_m,
                     double att_speed,
                     double acc_path,
                     const dto::Ammo& ammo) -> dto::DropSolution = 0;

  virtual auto solve(const pointmath::Point& drone_position, const pointmath::Point& target_position) -> dto::DropSolution = 0;

  virtual ~IBallisticSolver() = default;
};
