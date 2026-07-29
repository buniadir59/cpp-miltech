#pragma once

#include <string>

namespace dto {

struct SimStatistics {
  int total = 0;
  int active = 0;
  int underAttack = 0;
  int destroyed = 0;
  int firedCount = 0;
  int steps = 0;
  std::string ammoName;
  std::string solverName;

};

}  // namespace dto