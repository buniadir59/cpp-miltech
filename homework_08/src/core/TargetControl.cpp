#include "core/TargetControl.hpp"
#include "config/defines.hpp"

namespace core {

auto TargetControl::update(double tgtTimeStep) -> void
{
  speed = pointmath::getLength(now.delta) / tgtTimeStep;
}

auto TargetControl::getAccuracyS(double acc_m) -> double
{
  return speed > defines::kEps ? std::max(acc_m / speed, 0.1) : 0.1;
}

}  // namespace core