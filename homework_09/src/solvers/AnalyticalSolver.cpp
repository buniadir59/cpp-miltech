#include "solvers/AnalyticalSolver.hpp"
#include "dto/BallisticResult.hpp"
#include "dto/DropSolution.hpp"
#include "dto/BallisticsInput.hpp"
#include "math/point_math.hpp"

#include <cmath>
#include <stdexcept>

namespace {

constexpr double kGravity = 9.81;
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

}  // namespace

auto AnalyticalSolver::calculate_free_fall_time_s() const -> double
{
  const double drag_lift_speed = input.drag * input.lift * input.attack_speed;
  const double gravity_mass = kGravity * input.mass;
  const double a = input.drag * (gravity_mass - 2.0 * drag_lift_speed);  // a = d·g·m − 2d²·l·V₀
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

  double fall_time_s = 2.0 * std::cos((std::acos(acos_argument) + std::numbers::pi * 4.0) / 3.0) / sqrt_value - b_div_3a;
  return fall_time_s;
}

auto AnalyticalSolver::calculate_horizontal_fall_distance_m(double fall_time) const -> double
{
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

  double horizontal_fall_distance_m =
    fall_time * (input.attack_speed + fall_time * (a2 + fall_time * (a3 + fall_time * (a4 + fall_time * a5))));

  return horizontal_fall_distance_m;
}

void AnalyticalSolver::validate_input() const
{
  if ((input.mass <= kEpsilon) || (input.drag <= 0.0) || (input.lift < 0)) {
    throw std::invalid_argument("Ammo mass & drag must be positive, and lift must not be negative");
  }

  if (input.drone_z <= 0.0) {
    throw std::invalid_argument("Drone altitude must be positive");
  }

  if (input.attack_speed <= 0.0) {
    throw std::invalid_argument("Attack speed must be positive");
  }
}

auto AnalyticalSolver::solve(const pointmath::Point& drone_position, const pointmath::Point& target_position) -> dto::DropSolution
{
  // validate_input();
  dto::DropSolution result{};

  result.fall_time_s = calculate_free_fall_time_s();
  result.horizontal_fall_distance_m = calculate_horizontal_fall_distance_m(result.fall_time_s);

  if (result.horizontal_fall_distance_m < 0.0) {
    throw std::domain_error("Invalid horizontal fall distance");
  }

  const double minimum_distance_m = result.horizontal_fall_distance_m + input.acceleration_path;

  const pointmath::Point diff = target_position - drone_position;

  const double distance_to_target_m = pointmath::getLength(diff);

  if (distance_to_target_m < kEpsilon) {  // NB! solution is not optimal here, better to go in the direction opposite to target
    result.has_intermediate_point = true;
    result.interm_p = {target_position.x + minimum_distance_m, target_position.y};

    result.fire_p = {target_position.x + result.horizontal_fall_distance_m, target_position.y};

    return result;
  }

  if (minimum_distance_m > distance_to_target_m) {
    const double minimum_distance_ratio = minimum_distance_m / distance_to_target_m;

    result.has_intermediate_point = true;
    result.interm_p = target_position - diff * minimum_distance_ratio;
  }

  const double fire_point_ratio = (distance_to_target_m - result.horizontal_fall_distance_m) / distance_to_target_m;
  result.fire_p = drone_position + diff * fire_point_ratio;

  return result;
}

auto AnalyticalSolver::solve(const pointmath::Point& drone_position,
                             const pointmath::Point& target_position,
                             double altitude_m,
                             double att_speed,
                             double acc_path,
                             const dto::Ammo& ammo) -> dto::DropSolution
{
  input.setAmmoParams(ammo).setDroneAccelerationPath(acc_path).setDroneAltitude(altitude_m).setDroneAttackSpeed(att_speed);
  validate_input();
  if (input.acceleration_path <= 0.0) {
    throw std::invalid_argument("Acceleration path must be positive");
  }
  return solve(drone_position, target_position);
}

auto AnalyticalSolver::solveAmmo(double altitude_m, double att_speed, const dto::Ammo& ammo) -> dto::BallisticResult
{
  input.setAmmoParams(ammo).setDroneAltitude(altitude_m).setDroneAttackSpeed(att_speed);
  validate_input();
  dto::BallisticResult result{};
  result.ffTime = calculate_free_fall_time_s();
  result.hDist = calculate_horizontal_fall_distance_m(result.ffTime);
  return result;
}
