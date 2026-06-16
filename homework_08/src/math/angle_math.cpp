#include "math/angle_math.hpp"

#include <cmath>
#include <ostream>
/*
  Angle math lib
*/

namespace anglemath {

auto normalizeAngle(double value) -> double
{
  while (value > kPi) { 
    value -= 2 * kPi;
  }
  while (value <= - kPi) { 
    value += 2 * kPi; 
  }
  return value;
}

auto operator-(AngleRad a1, const AngleRad& a2) -> AngleRad
{
  a1 -= a2;
  return a1;
}

auto rad2Grad(double ang) -> int
{  // utility for better human presentation
  return static_cast<int>(round(ang / kPi * 180.0));
}

auto operator<<(std::ostream& os, const AngleRad& aR) -> std::ostream&
{
  return os << ",°: " << rad2Grad(aR.value);
}

}  // namespace anglemath