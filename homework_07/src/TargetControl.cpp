#include "TargetControl.hpp"
#include "defines.hpp"

namespace core {

auto TargetControl::update() -> void {
    speed = pointmath::getLength(now.velocity);   
   // accuracy_s = speed > defines::eps ? std::max(kAccuracy_m / speed, 0.1) : 0.1; 
}


auto TargetControl::getAccuracyS(double acc_m) -> double {
    return  speed > defines::eps ? std::max(acc_m / speed, 0.1) : 0.1;
}

}