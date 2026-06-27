#pragma once

#include "interfaces/ISimulationClock.hpp"

class ManualSimulationClock final : public ISimulationClock {
public:
  auto nowS() const -> double override;
  auto nowForTargetProvider() const -> double override;

  auto reset(double simTimeStep, double tgtTimeStep) -> void;

  auto advance() -> void;

private:
  double currentTimeS_{0.0};
  double tgtTimeStepS_{0.0};
  double simTimeStep_{0.0};
};
