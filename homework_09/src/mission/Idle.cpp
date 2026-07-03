#include "mission/Idle.hpp"
#include "mission/MissionCtx.hpp"
#include "mission/Attack.hpp"

#include <memory>
#include <cmath>

namespace mission {

// finds new target and check if it is reachable
// if found returns ptr to  respective state
// if not found returns nullptr
std::unique_ptr<IMissionState> Idle::execute(MissionCtx& ctx)
{
  int tag;
  while ((tag = ctx.getNextTarget()) >= 0) {
    // try to start new mission
    ctx.setCurrentTgtTag(tag);
    if (!ctx.calcAttackRoute()) {  // ok, we are attacking next target
      return std::make_unique<mission::Attack>();
    }
  }

  ctx.breakMission(); 
  return nullptr;      // continue idle, no need to check other states as we already know that hasNext is true
}

}  // namespace mission
