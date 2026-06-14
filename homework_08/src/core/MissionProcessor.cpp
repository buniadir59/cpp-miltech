#include "core/MissionProcessor.hpp"
#include "config/defines.hpp"
#include "dto/MissionConfig.hpp"
#include "dto/SimStatistics.hpp"
#include "dto/Target.hpp"
#include "core/TargetControl.hpp"
#include "core/DroneControl.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "math/angle_math.hpp"
#include "math/point_math.hpp"

#include <iterator>
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

auto MissionProcessor::init(const std::string& configSource) -> const dto::MissionConfig*
{
  if (!loader_->load(configSource)) {
    throw std::runtime_error("Error loading configuration");
  };

  j_out["steps"] = json::array();

  mconf = &(loader_->getConfig());
  ammo = &(loader_->getAmmoParams());

  drone.emplace(*mconf);
  mission.init(mconf->time_step, &*drone, mconf->tgt_time_step, ammo);

  const auto target_count = targets_->getTargetCount();
  targetDepo.assign(static_cast<std::size_t>(target_count), TargetControl{});

  for (auto& target : targetDepo) {  // initialise each tgt state
    target.state = ACTIVE;
  }

  currentTgtTag = 0;
  stepCurrent = 0;

  return &*mconf;
}

auto MissionProcessor::checkFireResult(TargetControl& tgt) -> bool
{
  if (std::abs(simClock->nowS() - tgt.hitTime) <= mconf->time_step / 2.0) {
    double dist = pointmath::getLength(tgt.now.position - tgt.hitCoord);
    LOG("@ Hit at dist=" << dist << "@ hit_time=" << tgt.hitTime);
    if (dist <= mconf->hit_rad) {
      tgt.state = core::DESTROYED;
    }
    else {
      tgt.state = core::ACTIVE;
    }
    return true;
  }
  return false;
}

auto MissionProcessor::getSimulationStatistics() -> dto::SimStatistics&
{
  stats.stepsTaken = stepCurrent;
  stats.destroyed = std::count_if(targetDepo.begin(), targetDepo.end(), [](const TargetControl& tgt) { return tgt.state == DESTROYED; });
  stats.underAttack = std::count_if(targetDepo.begin(), targetDepo.end(), [](const TargetControl& tgt) { return tgt.state == ATTACKED; });
  stats.total = targetDepo.size();
  stats.active = stats.total - stats.destroyed - stats.underAttack;

  return stats;
}

// gets new targets position and velocity
auto MissionProcessor::updateTargets() -> void
{
  for (std::size_t i = 0; i < targetDepo.size(); ++i) {  // update target position by index
    if (targetDepo[i].state == DESTROYED)
      continue;

    targetDepo[i].now = targets_->getTarget(i);
    targetDepo[i].update(mconf->tgt_time_step);

    switch (targetDepo[i].state) {
      case ACTIVE:
        break;

      case ATTACKED:
        if (checkFireResult(targetDepo[i])) {
          LOG(stepCurrent << "@ T#" << i << "@ time=" << simClock->nowS() << "@ Result of attack=" << targetDepo[i].targetStateToStr()
                          << "@ TPos@" << targetDepo[i].now.position << "@ TSpeed=" << targetDepo[i].speed);
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

  if (stepCurrent > defines::kMaxSteps) {  // simulation is over!
    return false;
  }

  updateTargets();  // unreachable => active

  if (mission.isOnMission()) {
    // analyze dron position on the drop path and change its state accordingly
    if (mission.continueMission()) {  // fired
      fire();
      mission.breakMission();  //@fired
    }
  }

  if (!mission.isOnMission()) {
    while (hasNext()) {
      if (currentTgtTag >= 0) {  // active target found @step
        DEBUG(stepCurrent << "@Try@ T#" << currentTgtTag << ' ' << targetDepo[currentTgtTag].targetStateToStr() << "@ TPos@"
                          << targetDepo[currentTgtTag].now.position << "@ TSpeed=" << targetDepo[currentTgtTag].speed
                          << "@ dist=" << pointmath::getLength(targetDepo[currentTgtTag].now.position - drone->coord)
                          << "@ angle=" << pointmath::getAngle(targetDepo[currentTgtTag].now.position - drone->coord));
        if (mission.startNewMission(targetDepo[currentTgtTag])) {  // go on mission
          break;
        }
      }
      else {  // no available targets, wait moving until farther (at minimum distance or more)
        drone->state = ACCELERATING;
        break;
      }
    }
  }  // eo is on mission

  pushStepToJSON();

  if (currentTgtTag >= 0) {
    DEBUG(stepCurrent << "@to T#" << currentTgtTag << " " << mission.missionStateToStr() << '@' << drone->droneStateToStr() << "@ Dr@"
                      << drone->coord << "@ Dir" << drone->dirRad << "@v=" << drone->speed << "@ TPos@"
                      << targetDepo[currentTgtTag].now.position << "@ Tv=" << targetDepo[currentTgtTag].speed << "@ tmr=" << mission.timer);
  }
  else {
    DEBUG(stepCurrent << "@ idle @ @" << drone->droneStateToStr() << "@ Dr@" << drone->coord << "@ Dir" << drone->dirRad
                      << "@ v=" << drone->speed);
  }

  ++stepCurrent;
  drone->move(mconf->time_step);
  return true;
}

auto MissionProcessor::getNextTarget() const -> int
{
  int start_idx = currentTgtTag < 0 ? 0 : currentTgtTag;

  auto it = std::find_if(targetDepo.begin() + start_idx, targetDepo.end(), [](const TargetControl& tgt) { return tgt.state == ACTIVE; });

  return it != targetDepo.end() ? std::distance(targetDepo.begin(), it) : -1;
}

// find and  set next Active target (current tgt tag)
// return false if all destroyed, true if active or under attack or unreachable tgts exist
auto MissionProcessor::identifyNextTarget() -> bool
{
  currentTgtTag = getNextTarget();
  if (currentTgtTag >= 0) {
    return true;
  }

  // start over
  reset();
  currentTgtTag = getNextTarget();
  if (currentTgtTag >= 0) {
    return true;
  }

  // no active targets, check if any of the targets are  Attacked or unreachable,
  // then wait for fire result, changing positions or end simulation
  auto it =
    std::ranges::find_if(targetDepo, [](const TargetControl& tgt) { return (tgt.state == ATTACKED) || (tgt.state == UNREACHABLE); });

  return it != targetDepo.end();
}

// Перевірити, чи є ще необроблені цілі
//  NB! for simulation will always return true
auto MissionProcessor::hasNext() -> bool
{
  if (!mission.isOnMission()) {
    return (targets_->getTargetCount() != 0) && (identifyNextTarget());
  }

  return true;
}

MissionProcessor::~MissionProcessor()
{
  j_out["totalSteps"] = stepCurrent;

  std::ofstream jf_out(defines::kSimulationPath);
  if (jf_out.is_open()) {
    jf_out << j_out.dump(2);  // 2 spaces => tab
  }
  // else { throw std::runtime_error("Unable to open output file: ");  }
}

// Записати дані кроку у вихідн. JSON файл
auto MissionProcessor::pushStepToJSON() -> void
{
  Point aimPoint = drone->getAimPoint(mission.ammoHorizDist);
  json step;  // крок х-дрона у-дрона кут-дрона стан-дрона ціль№
  step["position"] = {{"x", drone->coord.x}, {"y", drone->coord.y}};
  step["direction"] = static_cast<float>(drone->dirRad.value);
  step["state"] = drone->state;
  step["targetIndex"] = currentTgtTag;

  step["aimPoint"] = {{"x", aimPoint.x}, {"y", aimPoint.y}};

  if (currentTgtTag >= 0) {
    step["dropPoint"] = {{"x", mission.dropPoint.x}, {"y", mission.dropPoint.y}};

    step["predictedTarget"] = {{"x", mission.tgt_lead_pos.x}, {"y", mission.tgt_lead_pos.y}};
  }
  else {  // no target, the fields not defined
    step["dropPoint"] = nullptr;
    step["predictedTarget"] = nullptr;
  }
  j_out["steps"].push_back(step);
}

auto MissionProcessor::fire() -> void
{
  targetDepo[currentTgtTag].state = core::ATTACKED;
  targetDepo[currentTgtTag].hitCoord = drone->getAimPoint(mission.ammoHorizDist);
  targetDepo[currentTgtTag].hitTime = simClock->nowS() + mission.ammoFlyTime;
  LOG(stepCurrent << "@Fired! T#" << currentTgtTag << "@ hittime=" << targetDepo[currentTgtTag].hitTime << "@hitCoord@"
                  << targetDepo[currentTgtTag].hitCoord);
}

}  // namespace core
