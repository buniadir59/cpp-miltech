#include "point_math.hpp"

#include <cmath>

/*
  Point
*/
namespace pointmath {

namespace {
// angle tolerance  TODO: make part of Drone and recalculate ?
const double kAngleTolerance = std::atan(0.0003);  // = ( 0.1 * 3 / 1000 );
}  // namespace

auto operator+(Point a1, const Point& a2) -> Point
{
  a1 += a2;
  return a1;
}

auto operator-(Point a1, const Point& a2) -> Point
{
  a1 -= a2;
  return a1;
}

auto operator*(Point a1, double k) -> Point
{
  a1 *= k;
  return a1;
}

auto operator/(Point a1, double k) -> Point
{  // must to check k !=0 before calling!
  a1 /= k;
  return a1;
}

// **** transform angle to point of 1m radius
auto cossin(double a) -> Point
{
  return {std::cos(a), std::sin(a)};
}

auto getLength(const Point& A_B) -> double
{  // ***  returns length  of vector A_B
  return std::hypot(A_B.x, A_B.y);
}

// *** returns angle  of vector A_B
auto getAngle(const Point& A_B) -> double
{
  double angle = std::atan2(A_B.y, A_B.x);  // overloaded for Point
  return std::abs(angle) < kAngleTolerance ? 0.0 : angle;
}

/****
 * returns length and angle of vector A_B
 * if angle is negligible, returns 0
 */
auto trxPointToDistAngle(const Point& A_B, double& distance, double& angle) -> void
{
  distance = getLength(A_B);
  angle = getAngle(A_B);
}

auto fixNegativeZero(double val, int precision) -> double
{
  double absV = std::abs(val);
  return absV < 1 / pow(10, precision) ? 0.0 : val;
}

auto rad2Grad(double ang) -> int
{  // utility for better human presentation
  return static_cast<int>(round(ang / M_PI * 180));
}

auto operator<<(std::ostream& os, const Point& p) -> std::ostream&
{
  int prec = &os == &std::cout ? 1 : 2;
  return os << "( " << fixNegativeZero(p.x, prec) << ", " << fixNegativeZero(p.y, prec) << " )";
}
}  // namespace pointmath