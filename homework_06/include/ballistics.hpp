#ifndef HOMEWORK_06_BALLISTICS_HPP
#define HOMEWORK_06_BALLISTICS_HPP

#include <string>

namespace ballistics {

struct BallisticsInput {
    double drone_x = 0.0;
    double drone_y = 0.0;
    double drone_z = 0.0;

    double target_x = 0.0;
    double target_y = 0.0;

    double attack_speed = 0.0;
    double acceleration_path = 0.0;

    std::string ammo_name;
};

struct DropSolution {
    double fire_x = 0.0;
    double fire_y = 0.0;

    bool has_intermediate_point = false;
    double intermediate_x = 0.0;
    double intermediate_y = 0.0;
    
    double fall_time_s = 0.0;
    double horizontal_fall_distance_m = 0.0;
};

DropSolution compute_drop_solution(const BallisticsInput& input);

}  // namespace ballistics


#endif  // HOMEWORK_06_BALLISTICS_HPP