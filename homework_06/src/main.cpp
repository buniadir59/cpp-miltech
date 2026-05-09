#include "ballistics.hpp"

#include <exception>
#include <iostream>

int main()
{
    const ballistics::BallisticsInput input{
        .drone_x = 100.0,
        .drone_y = 100.0,
        .drone_z = 100.0,
        .target_x = 200.0,
        .target_y = 200.0,
        .attack_speed = 10.0,
        .acceleration_path = 10.0,
        .ammo_name = "VOG-17",
    };

    try {
        const ballistics::DropSolution solution =
            ballistics::compute_drop_solution(input);

        std::cout << "Drop point: " << solution.fire_x << ' ' << solution.fire_y
                  << '\n';
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }

    return 0;
}