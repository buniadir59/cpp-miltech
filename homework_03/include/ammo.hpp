#pragma once

#include <cstddef>

namespace ammo {

struct Ammo {
    char name[32]{};
    double mass{};
    double drag{};
    double lift{};
};

auto findAmmoByName(const Ammo ammo_table[], std::size_t ammo_count,
                    const char ammo_name[]) -> const Ammo*;

}  // namespace ammo