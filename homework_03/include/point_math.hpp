#pragma once

#include <iostream>
#include <cmath>
/*
  Point matyh lib
*/

namespace pointmath {
    
    struct Point { 
        double x = 0;
        double y = 0;
        Point() = default;
        Point(double x_val, double y_val) : x(x_val), y(y_val) {}; // Added constructor to avoid MSVC error

        Point& operator+=(const Point& delta) {  //increment
            x += delta.x;
            y += delta.y;
            return *this;
        }
        
        Point& operator-=(const Point& delta) {  //decrement
            x -= delta.x;
            y -= delta.y;
            return *this;
        }    
        
        Point& operator*=(const double& k) { //multiply by scalar
            x *= k;
            y *= k;
            return *this;
        }
        
        Point& operator/=(const double& k) { //divide by scalar  // check k !=0 before calling!
            x /= k;
            y /= k;
            return *this;
        } 

        bool operator==(const Point& other) const { //TODO: eps doesnt make sense - it shall be accuracy ...
            constexpr double eps = 1e-9;
            return std::abs(x - other.x) < eps && std::abs(y - other.y) < eps;
        }
    };

    Point operator+(Point a1, const Point& a2);        
    Point operator-(Point a1, const Point& a2); 
    Point operator*(Point a1, double k);
    Point operator/(Point a1, double k);  // check k !=0 before calling!
    
    
           
    auto fixNegativeZero(double val, int precision) -> double; 

    auto rad2Grad(double ang) -> int;
 
    Point cossin(double a); // transform angle to point of 1m radius
    
    double getLength(const Point&A_B);
    double getAngle(const Point&A_B);

    // returns length and angle of vector A_B. if angle is negligible, returns 0
    void trxPointToDistAngle(const Point&A_B, double& distance, double& angle);

    std::ostream& operator<<(std::ostream& os, const Point& p);
}