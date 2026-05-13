#pragma once

#include <numbers>

/*
  Point
  AngleRad
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

}