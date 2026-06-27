#pragma once

#include "dto/BallisticResult.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "dto/BallisticsInput.hpp"


class AnalyticalSolver : public IBallisticSolver {
  void validate_input() const;
  auto calculate_horizontal_fall_distance_m(double fall_time) const -> double;
  auto calculate_free_fall_time_s() const -> double;
  dto::BallisticsInput input;  // static

public:
  auto solve(const pointmath::Point& drone_position, const pointmath::Point& target_position)  -> dto::DropSolution override;

  auto solve(const pointmath::Point& drone_position,
             const pointmath::Point& target_position,
             double altitude_m,
             double att_speed,
             double acc_path,
             const dto::Ammo& ammo)  -> dto::DropSolution override;
 auto solveAmmo(double altitude_m,
                     double att_speed,
                     const dto::Ammo& ammo)  -> dto::BallisticResult override;             
};