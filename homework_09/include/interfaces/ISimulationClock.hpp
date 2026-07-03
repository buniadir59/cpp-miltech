#pragma once

class ISimulationClock {
public:
  virtual double nowS() const = 0;
  [[nodiscard]] virtual auto nowForTargetProvider() const -> double = 0;

  virtual ~ISimulationClock() = default;
};
