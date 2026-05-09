#include "ballistics.hpp"
#include <gtest/gtest.h>
#include <stdexcept>

namespace {

TEST(Ballistics, ComputesTemporaryDropPoint)
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

    EXPECT_NEAR(solution.fire_x, 200.0, 0.01);
    EXPECT_NEAR(solution.fire_y, 200.0, 0.01);
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