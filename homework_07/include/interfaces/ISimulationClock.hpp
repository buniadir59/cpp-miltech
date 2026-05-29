#pragma once

class ISimulationClock {
 public:
  virtual auto nowS() const -> double = 0;

  virtual ~ISimulationClock() = default;
};

 // namespace hw7::interfacesnamespace hw7::interfaces {