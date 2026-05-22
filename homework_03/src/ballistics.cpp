#include "ballistics.hpp"

#include <stdexcept>

#include "ballistics.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <array>

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
  AmmoType type;  // Kept as part of the original HW1 ammo table/domain model
};

constexpr std::array<Ammo, 5> kKnownAmmos{{
  {"VOG-17", 0.35, 0.07, 0.0, FreeFall},
  {"M67", 0.6, 0.10, 0.0, FreeFall},
  {"RKG-3", 1.2, 0.10, 0.0, FreeFall},
  {"GLIDING-VOG", 0.45, 0.10, 1.0, Gliding},
  {"GLIDING-RKG", 1.4, 0.10, 1.0, Gliding},
}};

auto find_ammo(const std::string& ammo_name) -> const Ammo&
{
  for (const auto& ammo : kKnownAmmos) {
    if (ammo_name == ammo.title) {
      return ammo;
    }
  }

  throw std::invalid_argument("Unknown ammo: " + ammo_name);
}

auto calculate_free_fall_time_s(const BallisticsInput& input, const Ammo& ammo) -> double
{
  const double drag_lift_speed = ammo.drag * ammo.lift * input.attack_speed;
  const double gravity_mass = kGravity * ammo.mass_kg;
  const double a = ammo.drag * (gravity_mass - 2.0 * ammo.drag * drag_lift_speed);
  const double b_div_3 = ammo.mass_kg * (drag_lift_speed - gravity_mass);
  const double c = input.drone_z * ammo.mass_kg * ammo.mass_kg * 6.0;

  if (std::abs(a) < kEpsilon) {
    if (std::abs(b_div_3) < kEpsilon) {
      throw std::domain_error("Unable to calculate fall time");
    }
    const double sqrt_arg = c / (-b_div_3 * 3.0);
    return std::sqrt(sqrt_arg);
  }

  const double b_div_3a = b_div_3 / a;
  const double b_div_3a_squared = b_div_3a * b_div_3a;

  const double p = -b_div_3a_squared * 3.0;
  const double q = b_div_3a_squared * b_div_3a * 2.0 + c / a;

  const double sqrt_value = std::sqrt(-3.0 / p);
  const double acos_argument = 1.5 * q / p * sqrt_value;

  if (std::abs(acos_argument) > 1.0) {
    throw std::domain_error("Simplified method for calculating fall time is not applicable");
  }

  const double res = 2.0 * std::cos((std::acos(acos_argument) + std::numbers::pi * 4.0) / 3.0) / sqrt_value - b_div_3a;
  ;

  return res;
}

auto calculate_horizontal_fall_distance_m(double fall_time_s, const BallisticsInput& input, const Ammo& ammo) -> double
{
  if (std::abs(ammo.mass_kg) < kEpsilon) {
    throw std::domain_error("Ammo mass must be non-zero");
  }

  const double drag_squared = ammo.drag * ammo.drag;
  const double drag_cubed = drag_squared * ammo.drag;

  const double lift_squared = ammo.lift * ammo.lift;
  const double lift_fourth = lift_squared * lift_squared;

  const double mass_squared = ammo.mass_kg * ammo.mass_kg;
  const double mass_cubed = mass_squared * ammo.mass_kg;
  const double mass_fourth = mass_cubed * ammo.mass_kg;

  const double one_plus_lift_squared = 1.0 + lift_squared;
  const double one_plus_lift_squared_squared = one_plus_lift_squared * one_plus_lift_squared;

  const double a2 = -(ammo.drag * input.attack_speed) / (2.0 * ammo.mass_kg);

  const double a3 =
    ammo.drag * (kGravity * ammo.lift * ammo.mass_kg - ammo.drag * (lift_squared - 1.0) * input.attack_speed) / (6.0 * mass_squared);

  const double a4 = (-6.0 * kGravity * drag_squared * ammo.lift * (one_plus_lift_squared + lift_fourth) * ammo.mass_kg +
                     3.0 * drag_cubed * lift_squared * one_plus_lift_squared * input.attack_speed +
                     6.0 * drag_cubed * lift_fourth * one_plus_lift_squared * input.attack_speed) /
                    (36.0 * one_plus_lift_squared_squared * mass_cubed);

  const double a5 =
    drag_cubed *
    (kGravity * ammo.lift * lift_squared * ammo.mass_kg - ammo.drag * lift_squared * one_plus_lift_squared * input.attack_speed) /
    (12.0 * one_plus_lift_squared * mass_fourth);

  return fall_time_s * (input.attack_speed + fall_time_s * (a2 + fall_time_s * (a3 + fall_time_s * (a4 + fall_time_s * a5))));
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

  if (input.acceleration_path <= 0.0) {
    throw std::invalid_argument("Acceleration path must be positive");
  }
}

}  // namespace

auto compute_drop_solution(const BallisticsInput& input) -> DropSolution
{
  validate_input(input);

  const Ammo& ammo = find_ammo(input.ammo_name);

  const double fall_time_s = calculate_free_fall_time_s(input, ammo);

  const double horizontal_fall_distance_m = calculate_horizontal_fall_distance_m(fall_time_s, input, ammo);

  if (horizontal_fall_distance_m < 0.0) {
    throw std::domain_error("Invalid horizontal fall distance");
  }

  const double minimum_distance_m = horizontal_fall_distance_m + input.acceleration_path;

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
    const double minimum_distance_ratio = minimum_distance_m / distance_to_target_m;

    solution.has_intermediate_point = true;
    solution.intermediate_x = input.target_x - dx * minimum_distance_ratio;
    solution.intermediate_y = input.target_y - dy * minimum_distance_ratio;
  }

  const double fire_point_ratio = (distance_to_target_m - horizontal_fall_distance_m) / distance_to_target_m;

  solution.fire_x = input.drone_x + dx * fire_point_ratio;
  solution.fire_y = input.drone_y + dy * fire_point_ratio;

  return solution;
}


  std::ostream& operator<<(std::ostream& os, const ballistics::DropSolution& ds) { 
      pointmath::Point iP = {ds.intermediate_x, ds.intermediate_y };
      pointmath::Point fP ={ds.fire_x, ds.fire_y};
      if (ds.has_intermediate_point) {
        pointmath::Point iP = {ds.intermediate_x, ds.intermediate_y };
        return os << " I" << iP << " F" << fP;
      }
      return os << " F" << fP;
  } 
}  // namespace ballistics