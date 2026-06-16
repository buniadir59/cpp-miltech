#include "solvers/TableSolver.hpp"
#include "dto/DropSolution.hpp"
#include "dto/BallisticsInput.hpp"
#include "math/point_math.hpp"

namespace {

//TODO constexpr double kGravity = 9.81;
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

}  // namespace

auto TableSolver::calculate_free_fall_time_s() const -> double
{
  
   return 0.0; //TODO
}

auto TableSolver::calculate_horizontal_fall_distance_m(double TableSolverTableSolver) const -> double
{
  
  return 0.0;
}

void TableSolver::validate_input() const
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

  if (input.acceleration_path <= 0.0) {
    throw std::invalid_argument("Acceleration path must be positive");
  }
}

auto TableSolver::solve(const pointmath::Point& drone_position, const pointmath::Point& target_position) -> dto::DropSolution
{
  validate_input();
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

auto TableSolver::solve(const pointmath::Point& drone_position,
                             const pointmath::Point& target_position,
                             double altitude_m,
                             double att_speed,
                             double acc_path,
                             const dto::Ammo& ammo) -> dto::DropSolution
{
  input.setAmmoParams(ammo).setDroneAccelerationPath(acc_path).setDroneAltitude(altitude_m).setDroneAttackSpeed(att_speed);
  return solve(drone_position, target_position);
}
