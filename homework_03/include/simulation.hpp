#pragma once

#include "point_math.hpp"
#include "drone.hpp"
#include "ammo.hpp"
#include <cstddef>

/* 
simulation  — SimConfig, TargetTrack, update target samples, main simulation mechanics

Simulation::
    updateTargetsPosition
*/

using size_t = std::size_t;
using Point = pointmath::Point;

namespace sim {   
    
    struct SimConfig {
        double time_step;        // simulation step (sec) 
        double tgt_time_step;    // target simulation step (sec)
        size_t number_of_targets = 0;
        size_t target_steps = 0;
        size_t number_of_ammos = 0;
        ammo::Ammo* ammo_table{nullptr};
        Point** target_tracks = nullptr;
    };

 
    // **** simulation engine data
    struct Simulation {
        size_t nTgts;          // number of targets 
        size_t targetSteps;    // length of simulation cycle for target coordinates
     //   size_t nAmmos;
        double timeStep;            // simulation step (sec) 
        double tgtTimeStep;         // target simulation step (sec)               
        Point** tgtTracks;
     //   ammo::Ammo* ammoTable{nullptr};
        drone::SimStep simStep{};

        explicit Simulation(const SimConfig& config)
        :   nTgts{config.number_of_targets},
            targetSteps{config.target_steps},
        //    nAmmos(config.number_of_ammos),
            timeStep{config.time_step},
            tgtTimeStep{config.tgt_time_step},          
            tgtTracks{config.target_tracks}
         //   ammoTable(config.ammo_table)         
        {}   

        auto moveTargets(double time_now, drone::Drone& dr) -> void; //receive "real" (interpolated) positions 

        auto initializeTgtPositions(drone::Drone& dr) -> void;
     
        auto getTgtPositionAt(int tgt_tag, double time_s) -> Point;

        auto freeMemory() -> void {
            if (tgtTracks != nullptr) {
                for (size_t i = 0; i < nTgts; ++i) {
                    delete[] tgtTracks[i];
                    tgtTracks[i] = nullptr;
                }

                delete[] tgtTracks;
                tgtTracks = nullptr;
            }
        }
    };

} //eo namespace sim 