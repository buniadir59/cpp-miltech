#include "config/ComponentFactory.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ISimulationClock.hpp"
#include "solvers/AnalyticalSolver.hpp"
#include "providers/JsonTargetProvider.hpp"
#include "config/FileConfigLoader.hpp"
#include "ManualSimulationClock.hpp"
#include "dto/MissionConfig.hpp"


ISimulationClock* ComponentFactory::createSimulationClock(SimulationClockType type) {
    switch (type) {
    case SimulationClockType::MANUAL:
        return  new ManualSimulationClock();

    default:
        return nullptr;
    }  
}


IBallisticSolver* ComponentFactory::createSolver(SolverType type) {
    switch (type) {
    case SolverType::ANALYTICAL:
        return  new AnalyticalSolver();

    default:
        return nullptr;
    }
}

ITargetProvider* ComponentFactory::createProvider(ProviderType type, const char* path) {

    switch (type) {
    case ProviderType::JSON:
        return new JsonTargetProvider(path);

    default:
        return nullptr;
    }
}
    
IConfigLoader* ComponentFactory::createLoader(LoaderType type) {
    switch (type) {
    case LoaderType::FILE:
        return new FileConfigLoader(); 

    default:
        return nullptr;
    }
}

void ComponentFactory::init(const dto::MissionConfig* mconf, ISimulationClock* simClock, ITargetProvider* tgtProv) {
    if (simClock == nullptr || tgtProv == nullptr) return;
    
    simClock -> reset(mconf->time_step,mconf->tgt_time_step);

    if (typeid(*tgtProv) == typeid(JsonTargetProvider)) {
        tgtProv -> init(simClock);
    }
    
    
}