#pragma once

namespace dto {
struct Ammo;
struct BallisticResult;
}  // namespace dto

class IBallisticSolver {
public:
  virtual auto solve(double altitude_m, double att_speed, const dto::Ammo& ammo) -> dto::BallisticResult = 0;
  virtual const char* name() const = 0;

  virtual ~IBallisticSolver() = default;
};
