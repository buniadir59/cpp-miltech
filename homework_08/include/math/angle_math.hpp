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

  AngleRad(double d = 0.0)
    : value(d)
  {
    normalize();
  }

  auto normalize() -> void
  {
    while (value > std::numbers::pi) {
      value -= 2 * std::numbers::pi;
    }
    while (value <= -std::numbers::pi) {
      value += 2 * std::numbers::pi;
    }
  }

  auto operator+=(double d) -> AngleRad&
  {
    value += d;
    normalize();
    return *this;
  }

  auto operator-=(double d) -> AngleRad&
  {
    value -= d;
    normalize();
    return *this;
  }

  auto operator+=(const AngleRad& d) -> AngleRad&
  {
    value += d.value;
    normalize();
    return *this;
  }

  auto operator-=(const AngleRad& d) -> AngleRad&
  {
    value -= d.value;
    normalize();
    return *this;
  }
};

auto operator-(AngleRad a1, const AngleRad& a2) -> AngleRad;

auto operator<<(std::ostream& os, const AngleRad& aR) -> std::ostream&;

}  // namespace anglemath