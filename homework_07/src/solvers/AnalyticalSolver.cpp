#include "solvers/AnalyticalSolver.h"

#include "ballistics.hpp"

namespace hw7::solvers {

auto AnalyticalSolver::solve(const pointmath::Point& drone_position,
                             const pointmath::Point& target_position,
                             double altitude_m,
                             const dto::Ammo& ammo) const -> dto::DropPoint {
  const auto solution =
      ballistics::computeDropPoint(drone_position, target_position, altitude_m,
                                   ammo);

  return dto::DropPoint{
      .position = solution.position,
  };
}

}  // namespace hw7::solvers