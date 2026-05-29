#include "ManualSimulationClock.hpp"



auto ManualSimulationClock::nowS() const -> double {
  return currentTimeS_;
}

void ManualSimulationClock::advance(double dt_s) {
  if (dt_s > 0.0) {
    currentTimeS_ += dt_s;
  }
}

void ManualSimulationClock::reset() {
  currentTimeS_ = 0.0;
}

 // namespace hw7::core