#pragma once

class ISimulationClock {
 public:
  virtual auto nowS() const -> double = 0;
  virtual auto nowForTargetProvider() const -> double = 0;

  virtual auto reset(double simTimeStep, double tgtTimeStep) -> void = 0;
  virtual auto advance() -> void = 0;

  virtual ~ISimulationClock() = default;
};
