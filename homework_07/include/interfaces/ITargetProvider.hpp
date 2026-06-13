#pragma once

#include "dto/Target.hpp"
#include "interfaces/ISimulationClock.hpp"

// Провайдер цілей: кількість та дані кожної цілі (позиція, швидкість)
class ITargetProvider {
public:
  virtual int getTargetCount() = 0;
  virtual dto::Target getTarget(int index) = 0;

  virtual auto init(const ISimulationClock* clock) -> void = 0;

  virtual ~ITargetProvider() {}
};