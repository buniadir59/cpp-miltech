#pragma once

namespace dto {

struct Ammo {
  char name[32]{};
  double mass{};
  double drag{};
  double lift{};
};

}  // namespace dto
