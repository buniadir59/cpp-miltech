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
  Point(double x_val, double y_val)
    : x(x_val)
    , y(y_val){};  // Added constructor to avoid MSVC error

  auto operator+=(const Point& delta) -> Point&
  {  // increment
    x += delta.x;
    y += delta.y;
    return *this;
  }

  auto operator-=(const Point& delta) -> Point&
  {  // decrement
    x -= delta.x;
    y -= delta.y;
    return *this;
  }

  auto operator*=(const double& k) -> Point&
  {  // multiply by scalar
    x *= k;
    y *= k;
    return *this;
  }

  auto operator/=(const double& k) -> Point&
  {  // divide by scalar  // check k !=0 before calling!
    x /= k;
    y /= k;
    return *this;
  }

  auto operator==(const Point& other) const
  {  // TODO: eps doesnt make sense - it shall be accuracy ...
    constexpr double eps = 1e-9;
    return std::abs(x - other.x) < eps && std::abs(y - other.y) < eps;
  }
};

auto operator+(Point a1, const Point& a2) -> Point;
auto operator-(Point a1, const Point& a2) -> Point;
auto operator*(Point a1, double k) -> Point;
auto operator/(Point a1, double k) -> Point;  // check k !=0 before calling!

auto fixNegativeZero(double val, int precision) -> double;

auto rad2Grad(double ang) -> int;

auto cossin(double a) -> Point;  // transform angle to point of 1m radius

auto getLength(const Point& A_B) -> double;
auto getAngle(const Point& A_B) -> double;

// returns length and angle of vector A_B. if angle is negligible, returns 0
auto trxPointToDistAngle(const Point& A_B, double& distance, double& angle) -> void;

auto operator<<(std::ostream& os, const Point& p) -> std::ostream&;
}  // namespace pointmath