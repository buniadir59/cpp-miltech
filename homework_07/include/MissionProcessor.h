#pragma once

#include "interfaces/ITargetProvider.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"

class MissionProcessor {

 public:
  MissionProcessor(ITargetProvider* targets,
                   IBallisticSolver* solver,
                   IConfigLoader* loader);

 private:
  ITargetProvider* targets_;      // borrowed, not owning
  IBallisticSolver* solver_;      // borrowed, not owning
  IConfigLoader* loader_;         // borrowed, not owning
  
};
