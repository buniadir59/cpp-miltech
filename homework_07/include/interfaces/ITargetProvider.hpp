#pragma once

#include "dto/Target.hpp"
#include "interfaces/ISimulationClock.hpp"

// Провайдер цілей: кількість та дані кожної цілі (позиція, швидкість)

// ITargetProvider additionally has init(clock), because target tracks are time-dependent
// and must be synchronized with the simulation clock.

class ITargetProvider {
public:
  virtual int getTargetCount() = 0;
  virtual dto::Target getTarget(int index) = 0;

  virtual auto init(const ISimulationClock* clock) -> void = 0;

  virtual ~ITargetProvider() = default;
};