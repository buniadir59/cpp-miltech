#include "solvers/TableSolver.hpp"
#include "dto/BallisticsInput.hpp"
#include "dto/BallisticResult.hpp"

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

auto TableSolver::solve(double altitude_m, double att_speed, const dto::Ammo& ammo) -> dto::BallisticResult
{
  input.setAmmoParams(ammo).setDroneAltitude(altitude_m).setDroneAttackSpeed(att_speed);
  validate_input();
  Result result = table.lookup({input.drone_z, input.attack_speed, input.mass, input.drag, input.lift});

  return dto::BallisticResult{result.ffTime, result.hDist};
}
