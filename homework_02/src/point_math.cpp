#include "point_math.hpp"

#include <cmath>


/*
  Point
  AngleRad
*/
namespace pointmath {
    
    AngleRad operator-(AngleRad a1, const AngleRad& a2) { 
            a1 -= a2;
            return a1;
    } 

    Point operator+(Point a1, const Point& a2) { 
            a1 += a2; 
            return a1; 
    }
        
    Point operator-(Point a1, const Point& a2) { 
            a1 -= a2;
            return a1;
    }

    Point operator*(Point a1, double k) { 
            a1 *= k;
            return a1;
    }

    Point operator/(Point a1, double k) {   // check k !=0 before calling!
            a1 /= k;
            return a1;
    } 

        // **** transform angle to point of 1m radius
        Point cossin(double a) { 
                return {std::cos(a), std::sin(a)};
        }

}