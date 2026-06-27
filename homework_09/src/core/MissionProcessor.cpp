#include "core/MissionProcessor.hpp"
#include "config/defines.hpp"
#include "dto/MissionConfig.hpp"
#include "dto/SimStatistics.hpp"
#include "dto/Target.hpp"
#include "core/TargetControl.hpp"
#include "mission/Idle.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "math/angle_math.hpp"
#include "math/point_math.hpp"
#include "config/defines.hpp"

#include <memory>
#include <nlohmann/json.hpp>
#include <cmath>
#include <stdexcept>
#include <fstream>
#include <optional>
#include <algorithm>
#include <string>

using json = nlohmann::json;

using Point = pointmath::Point;
using AngleRad = anglemath::AngleRad;

namespace core {

auto MissionProcessor::getSimulationStatistics() -> const dto::SimStatistics&
{
  stats.underAttack = std::count_if(targetDepo.begin(), targetDepo.end(), [](const TargetControl& tgt) { return tgt.state == ATTACKED; });
  stats.destroyed = std::count_if(targetDepo.begin(), targetDepo.end(), [](const TargetControl& tgt) { return tgt.state == DESTROYED; });
  stats.total = targetDepo.size();
  stats.active = stats.total - stats.destroyed - stats.underAttack;
  return stats;
}

auto MissionProcessor::init() -> const dto::MissionConfig*
{
  if (!loader_->load(defines::kConfigPath)) {
    throw std::runtime_error("Error loading configuration");
  };

  j_out["steps"] = json::array();

  mconf = &(loader_->getConfig());
  ammo = &(loader_->getAmmoParams());

  drone.emplace(*mconf);
  mctx.drone = &*drone;
  // set drone's copy of solver
  drone->setSolver(solver_.get());
  drone->setAmmo(ammo);

  mctx.tgtTimeStep = mconf->tgt_time_step;
  mctx.kAccuracy_m = mconf->time_step * mconf->attack_speed / 2.0;

  const auto target_count = targets_->getTargetCount();
  targetDepo.assign(static_cast<std::size_t>(target_count), TargetControl{});

  for (auto& target : targetDepo) {  // initialise each tgt state
    target.state = ACTIVE;
  }

  stats = {target_count, target_count, 0, 0, 0};

  mstate = std::make_unique<mission::Idle>();
  return &*mconf;
}

// gets new targets position and velocity
auto MissionProcessor::updateTargets() -> void
{
  for (std::size_t i = 0; i < targetDepo.size(); ++i) {  // up-date target position by index
    if (targetDepo[i].state == DESTROYED)
      continue;

    targetDepo[i].now = targets_->getTarget(i);
    targetDepo[i].update(mconf->tgt_time_step);

    switch (targetDepo[i].state) {
      case ACTIVE:
        break;

      case ATTACKED:

        if (std::fabs(simClock->nowS() - targetDepo[i].hitTime) <= mconf->time_step / 2.0) {
          stats.firedCount++;
          double dist = pointmath::getLength(targetDepo[i].now.position - targetDepo[i].hitCoord);
          if (dist <= mconf->hit_rad) {
            targetDepo[i].state = core::DESTROYED;
            stats.destroyed++;  // destroyed is final state
          }
          else {
            targetDepo[i].state = core::ACTIVE;
          }

          LOG(simClock->nowS() << " Result= _ " << targetDepo[i].targetStateToStr() << " _ _ Hit_at_dist " << dist << " _ T#" << i
                               << " TPos " << targetDepo[i].now.position << "_ _ _ TSpeed= " << targetDepo[i].speed);
        }

        break;

      case UNREACHABLE:
        targetDepo[i].state = ACTIVE;
        break;

      default:
        break;
    }
  }
}

// Логіка step():
// 1. Взяти наступну ціль через targets->get Target(currentIdx)
// 2. Викликати solver->solve(dronePos, target.pos, altitude, ammo)
// 3. Збільшити лічильник, повернути результат
// return false if #steps > max
bool MissionProcessor::step()
{
  if (!drone) {
    throw std::runtime_error("Drone data missing");
  }

  if (stats.steps > defines::kMaxSteps) {  // simulation is over!
    return false;
  }

  updateTargets();  // unreachable => active

  auto next = mstate->execute(mctx);
  if (next) {
    mstate = std::move(next);
  }
  pushStepToJSON();

  ++stats.steps;
  drone->execFly();
  return true;
}

// Перевірити, чи є ще необроблені цілі
//  NB! for simulation will always return true
auto MissionProcessor::hasNext() -> bool
{
  return stats.total != stats.destroyed;
}

MissionProcessor::~MissionProcessor()
{
  j_out["totalSteps"] = stats.steps;

  std::ofstream jf_out(defines::kSimulationPath);
  if (jf_out.is_open()) {
    jf_out << j_out.dump(2);  // 2 spaces => tab
  }
}

// Записати дані кроку у вихідн. JSON файл
auto MissionProcessor::pushStepToJSON() -> void
{
  Point aimPoint = drone->getInstantAimPoint();  // mission.getAmmoHDist());
  json step;                                     // крок х-дрона у-дрона кут-дрона стан-дрона ціль№
  step["position"] = {{"x", drone->getPosition().x}, {"y", drone->getPosition().y}};
  step["direction"] = static_cast<float>(drone->getDirection());
  step["state"] = drone->getStateName();
  step["targetIndex"] = mctx.currentTgtTag;

  step["aimPoint"] = {{"x", aimPoint.x}, {"y", aimPoint.y}};

  if (mctx.currentTgtTag >= 0) {
    step["dropPoint"] = {{"x", mctx.firePoint.x}, {"y", mctx.firePoint.y}};

    step["predictedTarget"] = {{"x", mctx.tgtLeadPos.x}, {"y", mctx.tgtLeadPos.y}};
  }
  else {                                             // no target, the fields not defined
    step["dropPoint"] = {{"x", 0}, {"y", 0}};        // to have same structure //nullptr;
    step["predictedTarget"] = {{"x", 0}, {"y", 0}};  // to have same structure //nullptr;
  }
  j_out["steps"].push_back(step);

  DEBUG(simClock->nowS() << " Pos " << drone->getPosition() << " Dir " << drone->getDirection() << " " << drone->getStateName() << " T#"
                         << mctx.currentTgtTag << " Aim " << aimPoint << " FP " << mctx.firePoint << " TLP " << mctx.tgtLeadPos << " "
                         << mstate->name());
}

}  // namespace core
