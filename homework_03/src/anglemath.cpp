#include "anglemath.hpp"

#include <cmath>

/*
  Angle math lib
*/

namespace anglemath {
    
    auto normalizeAngle(double value) -> double {
            while (value > std::numbers::pi ) {
                value -= 2 * std::numbers::pi ;
            }
            while (value <= -std::numbers::pi ) {
                value += 2 * std::numbers::pi ;
            }
            return value;
    }
    
    AngleRad operator-(AngleRad a1, const AngleRad& a2) { 
            a1 -= a2;
            return a1;
    } 

     auto rad2Grad(double ang) -> int {  //utility for better human presentation
        return static_cast<int>(round(ang/M_PI*180)); 
    }

    std::ostream& operator<<(std::ostream& os, const AngleRad& aR) { 
        return os << ",°: "<< rad2Grad(aR.value) << " ";
    }

}