#include "point_math.hpp"

#include <cmath>

/*
  Point
*/
namespace pointmath {

namespace  {
        //angle tolerance  TODO make part of Drone and recalculate ?
        const double kAngleTolerance = std::atan(0.0003); // = ( 0.1 * 3 / 1000 ); 
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

        double getLength(const Point&A_B) { // ***  returns length  of vector A_B
                return std::hypot(A_B.x, A_B.y); // hypot(A_B);
        }

        double getAngle(const Point&A_B) { // *** returns angle  of vector A_B
                double angle = std::atan2(A_B.y, A_B.x); //atan2(A_B); //overloaded for Point
                return std::abs(angle) < kAngleTolerance ? 0.0 : angle;
        }

        /****
        * returns length and angle of vector A_B
        * if angle is negligible, returns 0
        */
        void trxPointToDistAngle(const Point&A_B, double& distance, double& angle) { 
                distance = getLength(A_B);
                angle = getAngle(A_B); 
        }

        auto fixNegativeZero(double val, int precision) -> double {
                double absV = std::abs(val);
                return absV < 1 / pow(10, precision) ? 0.0 : val;
        }

        auto rad2Grad(double ang) -> int {  //utility for better human presentation
                return static_cast<int>(round(ang/M_PI*180)); 
        }

        std::ostream& operator<<(std::ostream& os, const Point& p) { 
                int prec = &os == &std::cout ? 1 : 2;
                return os << "( " << fixNegativeZero(p.x, prec) << ", " 
                                << fixNegativeZero(p.y, prec)  << " )";
        }
}