#pragma once

#include "math/point_math.hpp"

namespace dto {

struct MissionConfig {
  pointmath::Point drone_position{};
  double altitude{};
  double attack_speed{};
  double acceleration_path{};
  std::string ammo_name;
};

}