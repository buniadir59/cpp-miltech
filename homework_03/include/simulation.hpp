#pragma once

#include "point_math.hpp"
#include "drone.hpp"
#include "ammo.hpp"
#include <cstddef>

/* 
simulation  — SimConfig, TargetTrack, update target samples, 
main simulation mechanics

Simulation::
    owns:
        ammo::Ammo* ammoTable
        Point** targetTracks
    updateTargetsPosition
*/

using size_t = std::size_t;
using Point = pointmath::Point;

namespace sim {   
    
    struct SimConfig {
        double time_step;        // simulation step (sec) 
        double tgt_time_step;    // target simulation step (sec)
        double hit_rad;          //Радіус ураження — допустима похибка попадання (м)
    };

 
    // **** simulation engine data
    struct Simulation {
        size_t nTgts;          // number of targets 
        size_t nTargetSteps;   // length of simulation cycle for target coordinates
        size_t nAmmos;
        
        double timeStep;       // simulation step (sec) 
        double tgtTimeStep;    // target simulation step (sec)  
        const double kHitRad;         //Радіус ураження — допустима похибка попадання (м)  

        Point** tgtTracks = nullptr;          
        ammo::Ammo* ammoTable{nullptr};
        drone::SimStep simStep{};

        explicit Simulation(const SimConfig& config)
        :   timeStep{config.time_step},
            tgtTimeStep{config.tgt_time_step},
            kHitRad(config.hit_rad)    
        {}   
        
        // ## receive "real" (interpolated) positions each timestep //TODO optimize ??
        auto moveTargets(double time_now, drone::Drone& dr) -> void; 
        
        auto initializeTgtPositions(drone::Drone& dr) -> void;
     
        // ## get "real" position of target tgt_tag at any time_s 
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

            if (ammoTable != nullptr) {
                delete[] ammoTable;
                ammoTable = nullptr;
            }
        }
    };

} //eo namespace sim 