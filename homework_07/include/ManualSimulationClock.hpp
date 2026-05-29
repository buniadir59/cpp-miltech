#pragma once

#include "interfaces/ISimulationClock.hpp"

class ManualSimulationClock final : public ISimulationClock {
 public:
  auto nowS() const -> double override;
  
  void advance(double dt_s);
  void reset();

 private:
  double currentTimeS_{0.0};
};

  // namespace hw7::core namespace hw7::core {