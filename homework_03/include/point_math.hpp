#pragma once

#include <numbers>
#include <iostream>
#include <cmath>
/*
  Point
  AngleRad
*/

namespace pointmath {
    auto normalizeAngle(double value) -> double;
    
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

    };

    Point operator+(Point a1, const Point& a2);        
    Point operator-(Point a1, const Point& a2); 
    Point operator*(Point a1, double k);
    Point operator/(Point a1, double k);  // check k !=0 before calling!
    
    struct AngleRad {
        double value;

        AngleRad(double d = 0.0) : value(d) {
            normalize();
        }

        void normalize() {
            while (value > std::numbers::pi ) {
                value -= 2 * std::numbers::pi ;
            }
            while (value <= -std::numbers::pi ) {
                value += 2 * std::numbers::pi ;
            }
        }

        AngleRad& operator+=(double d) {
            value += d;
            normalize();
            return *this;
        }

        AngleRad& operator-=(double d) {
            value -= d;
            normalize();
            return *this;
        }

        AngleRad& operator+=(const AngleRad& d) {
            value += d.value;
            normalize();
            return *this;
        }

        AngleRad& operator-=(const AngleRad& d) {
            value -= d.value;
            normalize();
            return *this;
        }
    };

    AngleRad operator-(AngleRad a1, const AngleRad& a2);

           
    auto fixNegativeZero(double val, int precision) -> double; 

    auto rad2Grad(double ang) -> int;
 
    Point cossin(double a); // transform angle to point of 1m radius
    
    double getLength(const Point&A_B);
    double getAngle(const Point&A_B);

    // returns length and angle of vector A_B. if angle is negligible, returns 0
    void trxPointToDistAngle(const Point&A_B, double& distance, double& angle);

    std::ostream& operator<<(std::ostream& os, const pointmath::AngleRad& aR);  

    std::ostream& operator<<(std::ostream& os, const pointmath::Point& p);
}