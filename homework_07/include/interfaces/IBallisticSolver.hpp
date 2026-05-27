#pragma once

#include "dto/Ammo.hpp"
#include "dto/DropSolution.hpp"
#include "math/point_math.hpp"


class IBallisticSolver {
 public:
  virtual auto solve(const pointmath::Point& drone_position,
                     const pointmath::Point& target_position,
                     double altitude_m,
                     const dto::Ammo& ammo) const -> dto::DropSolution = 0;

  virtual ~IBallisticSolver() = default;
};

