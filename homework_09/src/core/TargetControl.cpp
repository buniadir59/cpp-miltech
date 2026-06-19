#include "core/TargetControl.hpp"


namespace {
  inline constexpr double kEps = 1e-9;
}

namespace core {

auto TargetControl::update(double tgtTimeStep) -> void
{
  speed = pointmath::getLength(now.delta) / tgtTimeStep;
}

auto TargetControl::getAccuracyS(double acc_m) -> double
{
  if (speed > kEps) {
    const double val = acc_m / speed; 
    return (val > 0.1) ? val : 0.1;
  }
  return 0.1;
}
}  // namespace core