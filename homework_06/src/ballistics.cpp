#include "ballistics.hpp"

#include <stdexcept>

#include "ballistics.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace ballistics {
namespace {

constexpr double kGravity = 9.81;
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

enum AmmoType {
    FreeFall,
    Gliding,
};

struct Ammo {
    const char* title;
    double mass_kg;
    double drag;
    double lift;
    AmmoType type; // Kept as part of the original HW1 ammo table/domain model
};

constexpr Ammo kKnownAmmos[] = {
    {"VOG-17", 0.35, 0.07, 0.0, FreeFall},
    {"M67", 0.6, 0.10, 0.0, FreeFall},
    {"RKG-3", 1.2, 0.10, 0.0, FreeFall},
    {"GLIDING-VOG", 0.45, 0.10, 1.0, Gliding},
    {"GLIDING-RKG", 1.4, 0.10, 1.0, Gliding},
};

const Ammo& find_ammo(const std::string& ammo_name)
{
    for (const auto& ammo : kKnownAmmos) {
        if (ammo_name == ammo.title) {
            return ammo;
        }
    }

    throw std::invalid_argument("Unknown ammo: " + ammo_name);
}

double calculate_free_fall_time_s(
    double attack_speed_mps,
    double altitude_m,
    double drag,
    double lift,
    double mass_kg)
{
    const double drag_lift_speed = drag * lift * attack_speed_mps;
    const double gravity_mass = kGravity * mass_kg;

    const double a = drag * (gravity_mass - 2.0 * drag * drag_lift_speed);
    const double b_div_3 = mass_kg * (drag_lift_speed - gravity_mass);
    const double c = altitude_m * mass_kg * mass_kg * 6.0;

    if (std::abs(a) < kEpsilon) {
        if (std::abs(b_div_3) < kEpsilon) {
            throw std::domain_error("Unable to calculate fall time");
        }

        return std::sqrt(c / (-b_div_3 * 3.0));
    }

    const double b_div_3a = b_div_3 / a;
    const double b_div_3a_squared = b_div_3a * b_div_3a;

    const double p = -b_div_3a_squared * 3.0;
    const double q = b_div_3a_squared * b_div_3a * 2.0 + c / a;

    const double sqrt_value = std::sqrt(-3.0 / p);
    const double acos_argument = 1.5 * q / p * sqrt_value;

    if (std::abs(acos_argument) > 1.0) {
        throw std::domain_error(
            "Simplified method for calculating fall time is not applicable");
    }

    const double phi = std::acos(acos_argument);

    return 2.0 * std::cos((phi + std::numbers::pi * 4.0) / 3.0) / sqrt_value
           - b_div_3a;
}

double calculate_horizontal_fall_distance_m(
    double fall_time_s,
    double attack_speed_mps,
    double drag,
    double lift,
    double mass_kg)
{
    if (std::abs(mass_kg) < kEpsilon) {
        throw std::domain_error("Ammo mass must be non-zero");
    }

    const double drag_squared = drag * drag;
    const double drag_cubed = drag_squared * drag;

    const double lift_squared = lift * lift;
    const double lift_fourth = lift_squared * lift_squared;

    const double mass_squared = mass_kg * mass_kg;
    const double mass_cubed = mass_squared * mass_kg;
    const double mass_fourth = mass_cubed * mass_kg;

    const double one_plus_lift_squared = 1.0 + lift_squared;
    const double one_plus_lift_squared_squared =
        one_plus_lift_squared * one_plus_lift_squared;

    const double a2 = -(drag * attack_speed_mps) / (2.0 * mass_kg);

    const double a3 =
        drag
        * (kGravity * lift * mass_kg
           - drag * (lift_squared - 1.0) * attack_speed_mps)
        / (6.0 * mass_squared);

    const double a4 =
        (-6.0 * kGravity * drag_squared * lift
             * (one_plus_lift_squared + lift_fourth) * mass_kg
         + 3.0 * drag_cubed * lift_squared * one_plus_lift_squared
               * attack_speed_mps
         + 6.0 * drag_cubed * lift_fourth * one_plus_lift_squared
               * attack_speed_mps)
        / (36.0 * one_plus_lift_squared_squared * mass_cubed);

    const double a5 =
        drag_cubed
        * (kGravity * lift * lift_squared * mass_kg
           - drag * lift_squared * one_plus_lift_squared * attack_speed_mps)
        / (12.0 * one_plus_lift_squared * mass_fourth);

    return fall_time_s
           * (attack_speed_mps
              + fall_time_s
                    * (a2
                       + fall_time_s
                             * (a3 + fall_time_s * (a4 + fall_time_s * a5))));
}

void validate_input(const BallisticsInput& input)
{
    if (input.ammo_name.empty()) {
        throw std::invalid_argument("Ammo name is empty");
    }

    if (input.drone_z <= 0.0) {
        throw std::invalid_argument("Drone altitude must be positive");
    }

    if (input.attack_speed <= 0.0) {
        throw std::invalid_argument("Attack speed must be positive");
    }

    if (input.acceleration_path < 0.0) {
        throw std::invalid_argument("Acceleration path must not be negative");
    }
}

}  // namespace

DropSolution compute_drop_solution(const BallisticsInput& input)
{
    validate_input(input);

    const Ammo& ammo = find_ammo(input.ammo_name);

    const double fall_time_s = calculate_free_fall_time_s(
        input.attack_speed,
        input.drone_z,
        ammo.drag,
        ammo.lift,
        ammo.mass_kg);

    const double horizontal_fall_distance_m =
        calculate_horizontal_fall_distance_m(
            fall_time_s,
            input.attack_speed,
            ammo.drag,
            ammo.lift,
            ammo.mass_kg);

    if (horizontal_fall_distance_m < 0.0) {
        throw std::domain_error("Invalid horizontal fall distance");
    }

    const double minimum_distance_m =
        horizontal_fall_distance_m + input.acceleration_path;

    const double dx = input.target_x - input.drone_x;
    const double dy = input.target_y - input.drone_y;
    const double distance_to_target_m = std::sqrt(dx * dx + dy * dy);

    DropSolution solution;
    solution.fall_time_s = fall_time_s;
    solution.horizontal_fall_distance_m = horizontal_fall_distance_m;

    if (distance_to_target_m < kEpsilon) {
        solution.has_intermediate_point = true;
        solution.intermediate_x = input.target_x + minimum_distance_m;
        solution.intermediate_y = input.target_y;

        solution.fire_x = input.target_x + horizontal_fall_distance_m;
        solution.fire_y = input.target_y;

        return solution;
    }

    if (minimum_distance_m > distance_to_target_m) {
        const double minimum_distance_ratio =
            minimum_distance_m / distance_to_target_m;

        solution.has_intermediate_point = true;
        solution.intermediate_x =
            input.target_x - dx * minimum_distance_ratio;
        solution.intermediate_y =
            input.target_y - dy * minimum_distance_ratio;
    }

    const double fire_point_ratio =
        (distance_to_target_m - horizontal_fall_distance_m)
        / distance_to_target_m;

    solution.fire_x = input.drone_x + dx * fire_point_ratio;
    solution.fire_y = input.drone_y + dy * fire_point_ratio;

    return solution;
}

}  // namespace ballistics