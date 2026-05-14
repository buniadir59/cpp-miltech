#pragma once

#include "point_math.hpp"
#include "drone.hpp"

#include <array>

/* 
simulation  — SimConfig, TargetTrack, update target samples, main simulation mechanics

Simulation::
    updateTargetsPosition
*/

namespace sim {
    struct SimConfig {
        double time_step;        // simulation step (sec) 
        double tgt_time_step;    // target simulation step (sec)
    };

    constexpr int kNtgts = 5;    //number of targets
    inline constexpr std::size_t kTargetSteps = 60; //length of simulation cycle for target coordinates
    
    struct TargetTrack {         //from targets.txt
        std::array<pointmath::Point, kTargetSteps> positions{};
    };

    // **** simulation engine data
    struct Simulation {
        double timeStep;        // simulation step (sec) 
        double tgtTimeStep;     // target simulation step (sec)               
    
        //**** targets positions simulation data  
        std::size_t last_sample_index = kTargetSteps; // != 0 => target positions will be updated when simulation starts
        std::array<sim::TargetTrack, 5> tgt_tracks{};

        explicit Simulation(const SimConfig& config)
        :   timeStep{config.time_step},
            tgtTimeStep{config.tgt_time_step}   
        {}   
        
        void resetTargetsPosition(drone::Drone& dr);
        void updateTargetsPosition(double timeCurrent, drone::Drone& dr);

    };

} //eo namespace sim 