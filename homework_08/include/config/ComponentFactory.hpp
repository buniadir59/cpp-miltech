#pragma once

#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ISimulationClock.hpp"

#include <memory>
#include <cstdint>

namespace dto {

struct MissionConfig;

}

class ComponentFactory {
public:
  enum class SolverType : std::uint8_t { ANALYTICAL };
  enum class ProviderType : std::uint8_t { JSON };
  enum class LoaderType : std::uint8_t { FILE };
  enum class SimulationClockType : std::uint8_t { MANUAL };

  std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
  std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const char* path);
  std::unique_ptr<IConfigLoader> createLoader(LoaderType type);
  std::unique_ptr<ISimulationClock> createSimulationClock(SimulationClockType type);

  void init(const dto::MissionConfig* mconf, ISimulationClock* simClock, ITargetProvider* tgtProv);
};