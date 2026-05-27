#pragma once

#include "interfaces/IBallisticSolver.hpp"


class AnalyticalSolver:IBallisticSolver {
 virtual auto solve(const pointmath::Point& drone_position,
                     const pointmath::Point& target_position,
                     double altitude_m,
                     const dto::Ammo& ammo) const -> dto::DropSolution
    
};