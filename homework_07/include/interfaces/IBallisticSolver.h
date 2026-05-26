#pragma once


#include "point_math.hpp"
#include "ammo.hpp"

class IBallisticSolver {

public:
    virtual int solve(
        const pointmath::Point& dronePos, 
        const pointmath::Point& targetPos, 
        double altitude, 
        const ammo::Ammo& ammo) = 0;

    virtual ~IBallisticSolver() {}

};