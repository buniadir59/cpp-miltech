#pragma once

#include <numbers>
#include <iostream>
#include <cmath>
/*
  AngleRad lib 
*/

namespace anglemath {

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

    std::ostream& operator<<(std::ostream& os, const AngleRad& aR);  

}