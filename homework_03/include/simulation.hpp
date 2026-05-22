#pragma once

#include "point_math.hpp"
#include "drone.hpp"

//#include <array>
#include <cstddef>

/* 
simulation  — SimConfig, TargetTrack, update target samples, main simulation mechanics

Simulation::
    updateTargetsPosition
*/

using size_t = std::size_t;
using Point = pointmath::Point;

namespace sim {
  
    struct Ammo {
        char name[32] = {0};
        float mass = 0.0; //kg
        float drag = 0.0; //коефіцієнт опору
        float lift = 0.0; //коефіцієнт підйому
    };    
    
    struct SimConfig {
        double time_step;        // simulation step (sec) 
        double tgt_time_step;    // target simulation step (sec)
        size_t number_of_targets = 0;
        size_t target_steps = 0;
        size_t number_of_ammos = 0;
        Ammo* ammo_table{nullptr};
        Point** target_tracks = nullptr;
    };

 
    // **** simulation engine data
    struct Simulation {
        size_t nTgts;          // number of targets 
        size_t targetSteps;    // length of simulation cycle for target coordinates
        size_t nAmmos;
        double timeStep;            // simulation step (sec) 
        double tgtTimeStep;         // target simulation step (sec)               
        Point** tgtTracks;
        Ammo* ammoTable{nullptr};
        //**** targets positions simulation data  
       // std::array<sim::TargetTrack, 5> tgt_tracks{};

        explicit Simulation(const SimConfig& config)
        :   nTgts{config.number_of_targets},
            targetSteps{config.target_steps},
            nAmmos(config.number_of_ammos),
            timeStep{config.time_step},
            tgtTimeStep{config.tgt_time_step},          
            tgtTracks{config.target_tracks},
            ammoTable(config.ammo_table)         
        {}   
        
   //     auto readAmmosJSON(const std::string& file_path, sim::Ammo* ammoTable)-> int;

        auto moveTargets(double time_now, drone::Drone& dr) -> void; //receive "real" (interpolated) positions 

        auto initializeTgtPositions(drone::Drone& dr) -> void;
     
        auto getTgtPositionAt(int tgt_tag, double time_s) -> Point;

        auto freeTargetTracks()->void { for (size_t i = 0; i < nTgts; ++i) { delete[] tgtTracks[i]; }; delete[] tgtTracks; };
    };

} //eo namespace sim 