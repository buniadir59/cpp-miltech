#include "mission/Idle.hpp"
#include "mission/MissionCtx.hpp"
#include "mission/Attack.hpp"

#include <memory>
#include <cmath>

namespace mission {

// finds new target and check if it is reachable
// if found returns ptr to  respective state
// if not found returns nullptr
std::unique_ptr<IMissionState> Attack::execute(MissionCtx& ctx)
{
  // re-calculate route to FP ()
  int res = ctx.calcAttackRoute();

  if (!res) {  // continue to FP

    return nullptr;
  }

  int tag;
  while ((tag = ctx.getNextTarget()) >= 0) {
    // try to start new mission
    ctx.setCurrentTgtTag(tag);
    res = ctx.calcAttackRoute();
    if (!res) {  // ok, we are attacking next target
      return nullptr;
    }
  }

  ctx.breakMission();  
  return std::make_unique<mission::Idle>();
}

}  // namespace mission
