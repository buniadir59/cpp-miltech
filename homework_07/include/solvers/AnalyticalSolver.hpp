#pragma once

#include "interfaces/IBallisticSolver.hpp"
#include "dto/BallisticsInput.hpp"



class AnalyticalSolver:IBallisticSolver {

    void validate_input() const;
    auto calculate_horizontal_fall_distance_m(double fall_time) const -> double;
    auto calculate_free_fall_time_s() const -> double;

    public:

        dto::BallisticsInput input; //static 

        virtual dto::DropSolution solve(const pointmath::Point& drone_position,
                     const pointmath::Point& target_position)  override;

        // dto::DropSolution solve(const pointmath::Point& drone_position,
        //              const pointmath::Point& target_position,
        //              double altitude_m, double att_speed, double acc_path,
        //              const dto::Ammo& ammo) const override;
  
};