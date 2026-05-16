#include "simulation.hpp"
#include "drone.hpp"
#include "point_math.hpp"

#include <cmath>
#include <cstddef>

/* 
simulation  — SimulationConfig, TargetTrack, update target samples, main simulation mechanics
*/

namespace sim {

namespace {  //for helpers


} //namespace for helpers

void Simulation::initializeTgtPositions(drone::Drone& dr) {
  last_sample_index = 0;
  for (auto i =0; i < sim::kNtgts; ++i) { 
        pointmath::Point pos = tgt_tracks[i].positions[0];
        dr.tgts[i].update(pos, 0.0);
  }
}

//obtaine and send current positions of targets to drone
auto Simulation::moveTargets(double time_now, drone::Drone& dr) -> void {
  const double ind_frac = time_now / tgtTimeStep;
  const double ind_double = std::floor(ind_frac);
  const std::size_t ind = static_cast<std::size_t>(ind_double) % kTargetSteps;
  const std::size_t ind_next = (ind + 1) % kTargetSteps;

  for (auto i =0; i < sim::kNtgts; ++i) { 
    sim::TargetTrack& tgt = tgt_tracks[i];
    pointmath::Point pos = (tgt.positions[ind_next] - tgt_tracks[i].positions[ind]) 
                          * (ind_frac - ind_double);
    pos += tgt_tracks[i].positions[ind];

    dr.tgts[i].update(pos, time_now); //send new position to drone
  }
}

}