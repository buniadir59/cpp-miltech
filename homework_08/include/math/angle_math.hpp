#pragma once

#include <iosfwd>
/*
  AngleRad lib
*/

namespace anglemath {

inline constexpr double kPi = 3.14159265358979323846;

struct AngleRad {
  double value;

  AngleRad(double d = 0.0)
    : value(d)
  {
    normalize();
  }

  auto normalize() -> void
  {
      while (value > kPi) { 
    value -= 2 * kPi; 
  }
  while (value <= - kPi) {  
    value += 2 * kPi;  
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