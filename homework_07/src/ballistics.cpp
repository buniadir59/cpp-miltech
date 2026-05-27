#include "ballistics.hpp"
#include "math/point_math.hpp"
#include <stdexcept>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace ballistics {

namespace {

constexpr double kGravity = 9.81;
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

double fall_time_s = 0.0;
double horizontal_fall_distance_m = 0.0;

struct {
  double drone_z = 0.0;
  double attack_speed = 0.0;
  double acceleration_path = 0.0;
  // ammo
  double mass = 0.0;
  double drag = 0.0;
  double lift = 0.0;
} FFInputLastUsed;

auto hasFFDataReady(const BallisticsInput& input)
{
  return input.drone_z == FFInputLastUsed.drone_z && input.attack_speed == FFInputLastUsed.attack_speed &&
         input.acceleration_path == FFInputLastUsed.acceleration_path && input.mass == FFInputLastUsed.mass &&
         input.drag == FFInputLastUsed.drag && input.drag == FFInputLastUsed.drag && input.lift == FFInputLastUsed.lift;
}

auto saveFFInput(const BallisticsInput& input)
{
  FFInputLastUsed.drone_z = input.drone_z;
  FFInputLastUsed.attack_speed = input.attack_speed;
  FFInputLastUsed.acceleration_path = input.acceleration_path;
  FFInputLastUsed.mass = input.mass;
  FFInputLastUsed.drag = input.drag;
  FFInputLastUsed.drag = input.drag;
  FFInputLastUsed.lift = input.lift;
}

auto calculate_free_fall_time_s(const BallisticsInput& input) -> double
{
  const double drag_lift_speed = input.drag * input.lift * input.attack_speed;
  const double gravity_mass = kGravity * input.mass;
  const double a = input.drag * (gravity_mass - 2.0 * input.drag * drag_lift_speed);
  const double b_div_3 = input.mass * (drag_lift_speed - gravity_mass);
  const double c = input.drone_z * input.mass * input.mass * 6.0;

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

  return res;
}

auto calculate_horizontal_fall_distance_m(double fall_time_s, const BallisticsInput& input) -> double
{
  if (std::abs(input.mass) < kEpsilon) {
    throw std::domain_error("Ammo mass must be non-zero");
  }

  const double drag_squared = input.drag * input.drag;
  const double drag_cubed = drag_squared * input.drag;

  const double lift_squared = input.lift * input.lift;
  const double lift_fourth = lift_squared * lift_squared;

  const double mass_squared = input.mass * input.mass;
  const double mass_cubed = mass_squared * input.mass;
  const double mass_fourth = mass_cubed * input.mass;

  const double one_plus_lift_squared = 1.0 + lift_squared;
  const double one_plus_lift_squared_squared = one_plus_lift_squared * one_plus_lift_squared;

  const double a2 = -(input.drag * input.attack_speed) / (2.0 * input.mass);

  const double a3 =
    input.drag * (kGravity * input.lift * input.mass - input.drag * (lift_squared - 1.0) * input.attack_speed) / (6.0 * mass_squared);

  const double a4 = (-6.0 * kGravity * drag_squared * input.lift * (one_plus_lift_squared + lift_fourth) * input.mass +
                     3.0 * drag_cubed * lift_squared * one_plus_lift_squared * input.attack_speed +
                     6.0 * drag_cubed * lift_fourth * one_plus_lift_squared * input.attack_speed) /
                    (36.0 * one_plus_lift_squared_squared * mass_cubed);

  const double a5 =
    drag_cubed *
    (kGravity * input.lift * lift_squared * input.mass - input.drag * lift_squared * one_plus_lift_squared * input.attack_speed) /
    (12.0 * one_plus_lift_squared * mass_fourth);

  return fall_time_s * (input.attack_speed + fall_time_s * (a2 + fall_time_s * (a3 + fall_time_s * (a4 + fall_time_s * a5))));
}

void validate_input(const BallisticsInput& input)
{
  if ((input.mass <= 0.0) || (input.drag <= 0.0) || (input.lift < 0)) {
    throw std::invalid_argument("Ammo mass & drag must be positive, and lift must not be negative");
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

  if (!hasFFDataReady(input)) {
    fall_time_s = calculate_free_fall_time_s(input);
    horizontal_fall_distance_m = calculate_horizontal_fall_distance_m(fall_time_s, input);
    if (horizontal_fall_distance_m < 0.0) {
      throw std::domain_error("Invalid horizontal fall distance");
    }
    saveFFInput(input);
  }

  const double minimum_distance_m = horizontal_fall_distance_m + input.acceleration_path;

  const Point diff = input.target_pos - input.drone_pos;

  const double distance_to_target_m = pointmath::getLength(diff);

  DropSolution solution;
  solution.fall_time_s = fall_time_s;
  solution.horizontal_fall_distance_m = horizontal_fall_distance_m;

  if (distance_to_target_m < kEpsilon) {  // TODO: solution is not optimal here, better to go in the direction opposite to target
    solution.has_intermediate_point = true;
    solution.interm_p = {input.target_pos.x + minimum_distance_m, input.target_pos.y};

    solution.fire_p = {input.target_pos.x + horizontal_fall_distance_m, input.target_pos.y};

    return solution;
  }

  if (minimum_distance_m > distance_to_target_m) {
    const double minimum_distance_ratio = minimum_distance_m / distance_to_target_m;

    solution.has_intermediate_point = true;
    solution.interm_p = input.target_pos - diff * minimum_distance_ratio;
  }

  const double fire_point_ratio = (distance_to_target_m - horizontal_fall_distance_m) / distance_to_target_m;
  solution.fire_p = input.drone_pos + diff * fire_point_ratio;

  return solution;
}

auto operator<<(std::ostream& os, const ballistics::DropSolution& ds) ->std::ostream&
{
  if (ds.has_intermediate_point) {
    return os << " I" << ds.interm_p << " F" << ds.fire_p;
  }
  return os << " F" << ds.fire_p;
}
}  // namespace ballistics