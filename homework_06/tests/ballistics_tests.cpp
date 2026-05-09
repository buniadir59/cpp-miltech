#include "ballistics.hpp"
#include <gtest/gtest.h>
#include <stdexcept>

namespace {

TEST(Ballistics, ComputesKnownVog17DropPoint)
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

    const ballistics::DropSolution solution =
        ballistics::compute_drop_solution(input);

    EXPECT_NEAR(solution.fire_x, 173.759, 0.01);
    EXPECT_NEAR(solution.fire_y, 173.759, 0.01);
    EXPECT_NEAR(solution.fall_time_s, 5.750, 0.01);
    EXPECT_NEAR(solution.horizontal_fall_distance_m, 37.110, 0.01);
    EXPECT_FALSE(solution.has_intermediate_point);
}

TEST(Ballistics, ThrowsForUnknownAmmo)
{
    const ballistics::BallisticsInput input{
        .drone_x = 100.0,
        .drone_y = 100.0,
        .drone_z = 100.0,
        .target_x = 200.0,
        .target_y = 200.0,
        .attack_speed = 10.0,
        .acceleration_path = 10.0,
        .ammo_name = "UNKNOWN",
    };

    EXPECT_THROW(
        static_cast<void>(ballistics::compute_drop_solution(input)),
        std::invalid_argument);
}

}  // namespace