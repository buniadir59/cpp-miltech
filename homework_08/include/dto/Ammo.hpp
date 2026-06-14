#pragma once
#include <string>

namespace dto {

struct Ammo {
  std::string name;
  double mass{};
  double drag{};
  double lift{};
};

}  // namespace dto
