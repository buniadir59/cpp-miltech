#include "simulation.hpp"
#include "drone.hpp"
#include "point_math.hpp"

#include <cmath>

/* 
    Simulation:: updateTargetsPosition  
*/

namespace sim {

namespace {  //for helpers


} //namespace for helpers


void Simulation::updateTargetsPosition(double timeCurrent, drone::Drone& dr) {
    const std::size_t sample_index =
        static_cast<int>(std::floor(timeCurrent / tgtTimeStep));

    if (sample_index != last_sample_index) {
      last_sample_index = sample_index;
      std::size_t index = sample_index % sim::kTargetSteps;
      
      for (auto i =0; i < sim::kNtgts; ++i) { 
        pointmath::Point pos = tgt_tracks[i].positions[index];
        dr.tgts[i].update(pos, timeCurrent);
      }
    }
}

}