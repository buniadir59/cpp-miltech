#include "config/ComponentFactory.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ISimulationClock.hpp"
#include "solvers/AnalyticalSolver.hpp"
#include "providers/JsonTargetProvider.hpp"
#include "config/FileConfigLoader.hpp"
#include "config/ManualSimulationClock.hpp"
#include "dto/MissionConfig.hpp"

#include <memory>
#include <string>

std::unique_ptr<ISimulationClock> ComponentFactory::createSimulationClock(SimulationClockType type)
{
  switch (type) {
    case SimulationClockType::MANUAL:
      return std::make_unique<ManualSimulationClock>();

    default:
      return nullptr;
  }
}

std::unique_ptr<IBallisticSolver> ComponentFactory::createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return std::make_unique<AnalyticalSolver>();

    default:
      return nullptr;
  }
}

std::unique_ptr<ITargetProvider> ComponentFactory::createProvider(ProviderType type, const std::string& path)
{
  switch (type) {
    case ProviderType::JSON:
      return std::make_unique<JsonTargetProvider>(path);

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

void ComponentFactory::init(const dto::MissionConfig* mconf, ISimulationClock* simClock, ITargetProvider* tgtProv)
{
  if (simClock == nullptr || tgtProv == nullptr)
    return;

  simClock->reset(mconf->time_step, mconf->tgt_time_step);

  if (typeid(*tgtProv) == typeid(JsonTargetProvider)) {
    tgtProv->init(simClock);
  }
}
