#include "drone.hpp"
#include "point_math.hpp"
//#include "ballistics.hpp"

#include <cmath>
//#include <limits>
//#include <numbers>
//#include <array>
//TODO #include <cstddef> // for std::size_t

/* **** drone.hpp / drone.cpp 
  TargetState::
    update target position
    predict target position
  DroneConfig 
  Drone::
    update drone params //TODO   
    choose target //TODO
  */

namespace drone {

namespace {  //for helpers


} //namespace for helpers

    void TargetState::update(pointmath::Point new_position, double time_s) { //update state with recent information about target location
        if (has_previous) {
            const double dt = time_s - last_known_s; //previous_time_s;
            velocity = (new_position - last_known) / dt;
        } else { //velocity stays 0
             has_previous = true;
        }

        last_known = new_position;
        last_known_s = time_s;
    }

    pointmath::Point TargetState::predictPositionAt(double future_t_s) const {
        return last_known + velocity * (future_t_s - last_known_s);
    }


 /*
  // **** calculates best target tag, assuming drone stopped & target are still 
  //      (= conditions at the beginning of simulation)
  void evaluateInitialTarget(drone::Drone& dr = sim.dr) {     
    int bestTag{-1}; //there will be tag of the target with the least time to hit
  //  double bestTT{std::numeric_limits<double>::max()};
    sim.input = {dr.coord.x, dr.coord.y, dr.alt, 0, 0,
      dr.attSpeed, dr.accPath, dr.ammoName};

    for (auto i=0; i < kNtgts; ++i) {
      sim.input.target_x = sim.tgt_tracks[i].positions[0].x;
      sim.input.target_y = sim.tgt_tracks[i].positions[0].y;
      sim.msns[i] = ballistics::compute_drop_solution(sim.input);
    }

    dr.currentTgtTag = bestTag;
  } */
/* 
  auto reevaluateBestTarget() {
  }

  auto steerDrone() {
  }

  auto updateDronePosition() {                           
  }
  
  auto startMission() {
  } 
*/

//##############################################################################################
    const std::string& getDroneStateStr(DroneState state) {
        static const std::string arrStateStr[] = { "STOPPED", "ACCELERATING", "DECELERATING", "TURNING", "MOVING",  "Unknown"};
        
        if (int(state) >= 5) {
            return arrStateStr[5];
        } else {
            return arrStateStr[state];
        }
    }    

} // drone