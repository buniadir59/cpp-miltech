#pragma once

#include "interfaces/ISimulationClock.hpp"

class ManualSimulationClock final : public ISimulationClock {
public:
  auto nowS() const -> double override;
  auto nowForTargetProvider() const -> double override;

  auto advance() -> void override;
  auto reset(double simTimeStep, double tgtTimeStep) -> void override;

private:
  double currentTimeS_{0.0};
  double tgtTimeStepS_{0.0};
  double simTimeStep_{0.0};
};
