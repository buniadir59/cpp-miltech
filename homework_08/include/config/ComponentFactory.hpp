#pragma once

#include <memory>
#include <cstdint>
#include <string>

class IBallisticSolver;
class ITargetProvider;
class IConfigLoader;

namespace dto {

struct MissionConfig;

}

class ComponentFactory {
public:
  enum class SolverType : std::uint8_t { ANALYTICAL };
  enum class ProviderType : std::uint8_t { JSON };
  enum class LoaderType : std::uint8_t { FILE };

  std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
  std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const std::string& path);
  std::unique_ptr<IConfigLoader> createLoader(LoaderType type);
};