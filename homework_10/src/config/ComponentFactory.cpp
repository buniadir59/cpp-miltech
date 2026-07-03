#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "solvers/AnalyticalSolver.hpp"
#include "solvers/TableSolver.hpp"
#include "providers/ThreadSafeTargetProvider.hpp"
#include "config/ComponentFactory.hpp"
#include "config/FileConfigLoader.hpp"

#include <memory>

std::unique_ptr<IBallisticSolver> ComponentFactory::createSolver(SolverType type, const std::string& path)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return std::make_unique<AnalyticalSolver>();
    case SolverType::TABLE:
      return std::make_unique<TableSolver>(path.c_str());
    default:
      return nullptr;
  }
}

std::unique_ptr<ITargetProvider> ComponentFactory::createProvider(ProviderType type,
                                                                  const std::string& path,
                                                                  const dto::MissionConfig& config)
{
  switch (type) {
    case ProviderType::JSON:
      return std::make_unique<ThreadSafeTargetProvider>(config, path);

    default:
      return nullptr;
  }
}

std::unique_ptr<IConfigLoader> ComponentFactory::createLoader(LoaderType type)
{
  switch (type) {
    case LoaderType::FILE:
      return std::make_unique<FileConfigLoader>();

    default:
      return nullptr;
  }
}
