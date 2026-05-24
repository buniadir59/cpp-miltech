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



auto Simulation::initializeTgtPositions(drone::Drone& dr) -> void {
 
  for (size_t i =0; i < nTgts; ++i) {    
        pointmath::Point pos = tgtTracks[i][0];
        dr.tgts[i].update(pos, 0.0);
  }
} 

auto Simulation::getTgtPositionAt(int tgt_tag, double time_s) -> pointmath::Point {
  const double ind_frac = time_s / tgtTimeStep;
  const double ind_double = std::floor(ind_frac);
  const std::size_t ind = static_cast<std::size_t>(ind_double) % nTargetSteps;
  const std::size_t ind_next = (ind + 1) % nTargetSteps;

  Point* track = tgtTracks[tgt_tag];
  Point pos = (track[ind_next] - track[ind]) 
                          * (ind_frac - ind_double);
  pos += track[ind];
  return pos;
}

//obtaine and send current positions of targets to drone
auto Simulation::moveTargets(double time_now, drone::Drone& dr) -> void {
  for (size_t i =0; i < nTgts; ++i) { 
    Point pos = getTgtPositionAt(i, time_now);

    dr.tgts[i].update(pos, time_now); //send new position to drone
  }
}

}