#include "dto/Ammo.hpp"

#include <cstring>
#include <stdexcept>

namespace ammo {

auto findAmmoByName(const dto::Ammo ammo_table[], std::size_t ammo_count, const char ammo_name[]) -> const dto::Ammo&
{
  for (std::size_t i = 0; i < ammo_count; ++i) {
    if (std::strcmp(ammo_table[i].name, ammo_name) == 0) {
      return ammo_table[i];
    }
  }

  throw std::runtime_error("Object not found");
}
}  // namespace ammo