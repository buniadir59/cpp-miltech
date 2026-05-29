#pragma once

#include "DropSolution.hpp"

namespace dto {

struct TargetState {  // current information on target available to drone

  double time_accuracy = 0.0;
  double time_total = 0.0;
  double time_to_interim = 0.0;  // time to reach interim point
  dto::DropSolution dropRoute{}; //TODO move somewhere???

  // return projected position for lead targeting
  //Point getLeadPosition(double delta_t) const { return last_known + velocity * (delta_t); };
    
  //TODO active/fired/destroyed state

};  // eo TargetState 
std::ostream& operator<<(std::ostream& os, const dto::TargetState& tgt);
}
