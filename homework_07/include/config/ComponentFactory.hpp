#pragma once

#include "interfaces/IBallisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"

class ComponentFactory {

public:
    enum class SolverType { ANALYTICAL };
    enum class ProviderType { JSON };
    enum class LoaderType { FILE };

    IBallisticSolver* createSolver(SolverType type);
    ITargetProvider* createProvider(ProviderType type, const char* param);
    IConfigLoader* createLoader(LoaderType type);

};