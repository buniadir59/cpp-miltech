#pragma once

#include <cstddef>

namespace dto {

struct Ammo {
  char name[32]{};
  double mass{};
  double drag{};
  double lift{};
};

}  //eo namespace dto

namespace ammo {

auto findAmmoByName(const dto::Ammo ammo_table[], std::size_t ammo_count, const char ammo_name[]) -> const dto::Ammo*;

}// namespace ammo