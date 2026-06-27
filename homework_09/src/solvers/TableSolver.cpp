#include "solvers/TableSolver.hpp"
#include "dto/DropSolution.hpp"
#include "dto/BallisticsInput.hpp"
#include "dto/BallisticResult.hpp"
#include "math/point_math.hpp"

#include <cmath>
#include <stdexcept>

namespace {
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

}  // namespace

TableSolver::TableSolver(const char* source)
{
  if (!table.load(source)) {
    throw std::runtime_error("Error reading ballistic table");
  }
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
}

auto TableSolver::solve(const pointmath::Point& drone_position, const pointmath::Point& target_position)  -> dto::DropSolution
{
  dto::DropSolution drop_route{};
  Result ball_res = table.lookup({input.drone_z, input.attack_speed, input.mass, input.drag, input.lift});
  drop_route.fall_time_s = ball_res.ffTime;
  drop_route.horizontal_fall_distance_m = ball_res.hDist;

  const double minimum_distance_m = ball_res.hDist + input.acceleration_path;

  const pointmath::Point diff = target_position - drone_position;
  const double distance_to_target_m = pointmath::getLength(diff);

  if (distance_to_target_m < kEpsilon) {  // NB! solution is not optimal here, better to go in the direction opposite to target
    drop_route.has_intermediate_point = true;
    drop_route.interm_p = {target_position.x + minimum_distance_m, target_position.y};

    drop_route.fire_p = {target_position.x + drop_route.horizontal_fall_distance_m, target_position.y};

    return drop_route;
  }

  if (minimum_distance_m > distance_to_target_m) {
    const double minimum_distance_ratio = minimum_distance_m / distance_to_target_m;

    drop_route.has_intermediate_point = true;
    drop_route.interm_p = target_position - diff * minimum_distance_ratio;
  }

  const double fire_point_ratio = (distance_to_target_m - drop_route.horizontal_fall_distance_m) / distance_to_target_m;
  drop_route.fire_p = drone_position + diff * fire_point_ratio;

  return drop_route;
}

auto TableSolver::solve(const pointmath::Point& drone_position,
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

auto TableSolver::solveAmmo(double altitude_m, double att_speed, const dto::Ammo& ammo) -> dto::BallisticResult
{
  input.setAmmoParams(ammo).setDroneAltitude(altitude_m).setDroneAttackSpeed(att_speed);
  validate_input();
  Result result = table.lookup({input.drone_z, input.attack_speed, input.mass, input.drag, input.lift});

  return dto::BallisticResult{result.ffTime, result.hDist};
}
