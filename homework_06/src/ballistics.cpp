#include "ballistics.hpp"

#include <stdexcept>

namespace ballistics {

DropSolution compute_drop_solution(const BallisticsInput& input)
{
    if (input.ammo_name.empty()) {
        throw std::invalid_argument("Ammo name is empty");
    }

    if (input.ammo_name != "VOG-17") {
        throw std::invalid_argument("Unknown ammo type");
    }

    if (input.drone_z <= 0.0) {
        throw std::invalid_argument("Drone altitude must be positive");
    }

    // Temporary stub.
    // Real ballistic calculation will be moved here from HW1.
    DropSolution solution;
    solution.fire_x = input.target_x;
    solution.fire_y = input.target_y;

    return solution;
}

}  // namespace ballistics