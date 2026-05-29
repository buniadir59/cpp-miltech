#pragma once

//#include "dto/Ammo.hpp"

#include <cstddef>
#include "math/point_math.hpp"

namespace dto {

struct MissionConfig {
  pointmath::Point drone_position{};
  double altitude{};
  double initial_direction{};
  double attack_speed{};
  double acceleration_path{};
  double angular_speed{};
  double turn_threshold{};
  double hit_rad{};
  double time_step{};
  double tgt_time_step{};
  size_t nAmmos = 0;
  //const dto::Ammo& ammo;
  
   int maxTargets = 5; //TODO ??
// SourceType loaderType;
// char loaderParam[256];
// bool useTableSolver;
};

}