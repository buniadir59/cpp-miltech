#include "core/TargetControl.hpp"
#include "math/point_math.hpp"

namespace {
inline constexpr double kEps = 1e-9;
inline constexpr double kMinTimeAcc = 0.1;
}  // namespace

namespace core {

//called every step to obtain new coordinates and velocity params  
auto TargetControl::update(double tgtTimeStep) -> void
{
  speed = pointmath::getLength(now.delta) / tgtTimeStep;
}

//returns time for which the target makes acc_m distance
auto TargetControl::getAccuracyS(double acc_m) -> double
{
  double val = kMinTimeAcc;
  if (speed > kEps) {
    val = acc_m / speed;
  }
  return (val > kMinTimeAcc) ? val : kMinTimeAcc;
}

auto TargetControl::getLeadPos(double lead_time_ratio) -> pointmath::Point
{
  return now.position + now.delta * lead_time_ratio;  //(lead_time / tgtTimeStep)
}

}  // namespace core