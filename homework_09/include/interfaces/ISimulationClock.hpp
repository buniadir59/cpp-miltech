#pragma once

class ISimulationClock {
public:
  [[nodiscard]] virtual auto nowS() const -> double = 0;
  [[nodiscard]] virtual auto nowForTargetProvider() const -> double = 0;

  virtual auto reset(double simTimeStep, double tgtTimeStep) -> void = 0;
  virtual auto advance() -> void = 0;

  virtual ~ISimulationClock() = default;
};
