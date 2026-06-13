#pragma once

#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ISimulationClock.hpp"
#include "dto/MissionConfig.hpp"


class ComponentFactory {

public:
    enum class SolverType { ANALYTICAL };
    enum class ProviderType { JSON };
    enum class LoaderType { FILE };
    enum class SimulationClockType {MANUAL};

    IBallisticSolver* createSolver(SolverType type);
    ITargetProvider* createProvider(ProviderType type, const char* path);
    IConfigLoader* createLoader(LoaderType type);
    ISimulationClock* createSimulationClock(SimulationClockType type);

    void init(const dto::MissionConfig* mconf, ISimulationClock* simClock, ITargetProvider* tgtProv);

   // ~ComponentFactory();
};