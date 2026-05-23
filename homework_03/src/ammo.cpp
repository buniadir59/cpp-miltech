#include "ammo.hpp"

#include <cstring>

namespace ammo {

auto findAmmoByName(const ammo::Ammo* ammo_table, std::size_t ammo_count,
                    const char ammo_name[]) -> const Ammo* {
    for (std::size_t i = 0; i < ammo_count; ++i) {
        if (std::strcmp(ammo_table[i].name, ammo_name) == 0) {
            return &ammo_table[i];
        }
    }

    return nullptr;
}
/*
auto findAmmoByName(const Ammo ammo_table[], std::size_t ammo_count,
                    const char ammo_name[]) -> const Ammo* {
    for (std::size_t i = 0; i < ammo_count; ++i) {
        if (std::strcmp(ammo_table[i].name, ammo_name) == 0) {
            return &ammo_table[i];
        }
    }

    return nullptr;
}*/
}  // namespace ammo