#include "config/ManualSimulationClock.hpp"
#include <stdexcept>
#include "config/defines.hpp"

auto ManualSimulationClock::nowS() const -> double
{
  return currentTimeS_;
}

auto ManualSimulationClock::nowForTargetProvider() const -> double
{
  if (tgtTimeStepS_ < defines::kEps) {
    throw std::runtime_error("Time step for target provider not provided");
  }

  return currentTimeS_ / tgtTimeStepS_;
}

void ManualSimulationClock::advance()
{
  currentTimeS_ += simTimeStep_;
}

void ManualSimulationClock::reset(double simTimeStep, double tgtTimeStep)
{
  currentTimeS_ = 0.0;
  simTimeStep_ = simTimeStep;
  tgtTimeStepS_ = tgtTimeStep;
}
