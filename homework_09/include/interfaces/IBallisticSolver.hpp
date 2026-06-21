#pragma once

//#include "dto/BallisticResult.hpp"
//#include "dto/DropSolution.hpp"
//#include "dto/Ammo.hpp"
#include "math/point_math.hpp"

namespace dto {
  struct Ammo;
  struct BallisticResult;
  struct DropSolution;
}
class IBallisticSolver {
public:
  virtual auto solve(const pointmath::Point& drone_position,
                     const pointmath::Point& target_position,
                     double altitude_m,
                     double att_speed,
                     double acc_path,
                     const dto::Ammo& ammo) -> dto::DropSolution = 0;

  virtual auto solve(const pointmath::Point& drone_position, const pointmath::Point& target_position) -> dto::DropSolution = 0;
  virtual auto solveAmmo(double altitude_m,
                     double att_speed,
                     const dto::Ammo& ammo) -> dto::BallisticResult = 0;
  virtual ~IBallisticSolver() = default;
};
