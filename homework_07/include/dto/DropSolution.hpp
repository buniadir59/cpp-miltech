#pragma once

#include "math/point_math.hpp"

namespace dto {

struct DropSolution {
  pointmath::Point fire_p{};
  pointmath::Point interm_p{};
  bool has_intermediate_point = false;
  double fall_time_s = 0.0;
  double horizontal_fall_distance_m = 0.0;
};

}  // namespace dto
