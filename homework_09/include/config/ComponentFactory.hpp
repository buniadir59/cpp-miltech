#pragma once

#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include <memory>
#include <cstdint>
#include <string>


namespace dto {

struct MissionConfig;

}

class ComponentFactory {
public:
  enum class SolverType : std::uint8_t { ANALYTICAL, TABLE };
  enum class ProviderType : std::uint8_t { JSON };
  enum class LoaderType : std::uint8_t { FILE };

  std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
  std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const std::string& path);
  std::unique_ptr<IConfigLoader> createLoader(LoaderType type);
};