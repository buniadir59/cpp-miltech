#pragma once

#include "point_math.hpp"

using Point = pointmath::Point;

namespace ballistics {

struct BallisticsInput {
  Point drone_pos {};
  double drone_z = 0.0;

  Point target_pos{};

  double attack_speed = 0.0;
  double acceleration_path = 0.0;

  // ammo 
  double mass = 0.0;
  double drag = 0.0;
  double lift = 0.0;
};

struct DropSolution {
  Point fire_p{};
  Point interm_p{};
  bool has_intermediate_point = false;  
  double fall_time_s = 0.0;
  double horizontal_fall_distance_m = 0.0;
};

std::ostream& operator<<(std::ostream& os, const ballistics::DropSolution& ds);

auto compute_drop_solution(const BallisticsInput& input) -> DropSolution;

}  // namespace ballistics
