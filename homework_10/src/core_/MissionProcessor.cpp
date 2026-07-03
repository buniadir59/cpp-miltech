#include "drone/DronePhysics.hpp"
#include "core_/MissionProcessor.hpp"
#include "core_/TargetControl.hpp"
#include "core_/TimeTracker.hpp"
#include "dto/Ammo.hpp"
#include "dto/BallisticResult.hpp"
#include "dto/SimStatistics.hpp"
#include "dto/Target.hpp"
#include "mission/Idle.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "math/angle_math.hpp"
#include "math/point_math.hpp"
#include "config/defines.hpp"  //for LOG/DEBUG

#include <memory>
#include <nlohmann/json.hpp>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <string>
#include <thread>

using json = nlohmann::json;

using Point = pointmath::Point;
using AngleRad = anglemath::AngleRad;

namespace core {

auto MissionProcessor::run() -> void
{
  init();
  auto init() -> void;  // Завантажити конфіг, підготувати дані ітерації
  threadReady = true;

  while (!threadStart) {
  };

  TimeTracker& tt = TimeTracker::getInstance();
  double time = 0.0;
  int step_now = 0;
  LOG("Started " << tt.getElapsed());

  while (true) {//} (!hasNext()) {
    if (!step()) {
      LOG("Simulation_time_is_over!");  // TODO result as bool
      break;
    };
    auto wakeup = tt.nextWakeup(timeStep);
    std::this_thread::sleep_until(wakeup);
    LOG("wakeup=" <<  tt.getElapsed()); 
  }

    LOG("Ended " << tt.getElapsed());
}

// checks what if we fire now()
// returns true if fired
/* auto MissionProcessor::_checkFireCondition() -> bool
{
  pointmath::Point tlp = _getCurrTgtLeadPos(drone->getInstantAmmoFFTime());  //@cfcond
  pointmath::Point aim_p = drone->getInstantAimPoint();
  double hit_dist = pointmath::getLength(aim_p - tlp);
  if (hit_dist <= kAccuracy_m) {  // Fire
    firePoint = drone->getPosition();
    currTgt->state = core::ATTACKED;
    currTgt->hitCoord = drone->getInstantAimPoint();
    currTgt->hitTime = TimeTracker::getInstance().getElapsed() + drone->getInstantAmmoFFTime();
    LOG("H=>" << currTgt->hitTime << " _ _ _ _ _ _ _ Fired! T#" << currentTgtTag << " hitXY " << currTgt->hitCoord 
     // << " _ _ _ _ _ hitXY " << currTgt->hitCoord
    );

    return true;
  }
  return false;
} */

// Логіка step():
// 1. Взяти наступну ціль через targets->get Target(currentIdx)
// 2. Викликати solver->solve(dronePos, target.pos, altitude, ammo)
// 3. Збільшити лічильник, повернути результат
// return false if #steps > max
bool MissionProcessor::step()
{
  if (stats.steps > kMaxSteps) {  // simulation is over!
    return false;
  }
  /*  TODO restore
    updateTargets();  // unreachable => active
    drone::DroneTelemetry telemetry = drone_.getTelemetry();
    auto next = mstate->execute(mctx);
    if (next) {
      mstate = std::move(next);
    }
    pushStepToJSON(telemetry); */

  ++stats.steps;
  //  drone->execFly();
  return true;
}

// Перевірити, чи є ще необроблені цілі
//  NB! for simulation will always return true
auto MissionProcessor::hasNext() -> bool
{
  return stats.total != stats.destroyed;
}

auto MissionProcessor::getSimulationStatistics() -> const dto::SimStatistics&
{
  stats.underAttack = std::count_if(targetDepo.begin(), targetDepo.end(), [](const TargetControl& tgt) { return tgt.state == ATTACKED; });
  stats.destroyed = std::count_if(targetDepo.begin(), targetDepo.end(), [](const TargetControl& tgt) { return tgt.state == DESTROYED; });
  stats.total = targetDepo.size();
  stats.active = stats.total - stats.destroyed - stats.underAttack;
  return stats;
}

// gets new targets position and velocity
auto MissionProcessor::updateTargets() -> void
{
  for (std::size_t i = 0; i < targetDepo.size(); ++i) {  // up-date target position by index
    if (targetDepo[i].state == DESTROYED)
      continue;

    targetDepo[i].now = targets_.getTarget(i);
    targetDepo[i].update();

    switch (targetDepo[i].state) {
      case ACTIVE:
        break;

      case ATTACKED:
        if ((TimeTracker::getInstance().getElapsed() - targetDepo[i].hitTime) <
            0.05) {  // TODO replace with tgt time step // mconf->timeStep / 2.0
          stats.firedCount++;
          double dist = pointmath::getLength(targetDepo[i].now.position - targetDepo[i].hitCoord);
          if (dist <= 3.0) {  // TODO somehow replace magic number with ?? } mconf->hitRadius) {
            targetDepo[i].state = core::DESTROYED;
            stats.destroyed++;  // destroyed is final state
          }
          else {
            targetDepo[i].state = core::ACTIVE;
          }

          LOG(TimeTracker::getInstance().getElapsed()
              << " Result= _ " << targetDepo[i].targetStateToStr() << " _ _ Hit_at_dist " << dist << " _ T#" << i << " TPos "
              << targetDepo[i].now.position << "_ _ _ TSpeed= " << targetDepo[i].speed);
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

auto MissionProcessor::init() -> void
{
  j_out["steps"] = json::array();

  // mconf = &(loader_->getConfig());
  // ammo = &(loader_->getAmmoParams());

  // TODO move  drone.emplace(*mconf);
  //??? mctx.drone = &*drone;
  /*   // set drone's copy of solver
    drone->setSolver(solver_.get());
    drone->setAmmo(ammo); */

  mctx.kAccuracy_m = 0.5;  // TODO somehow replace magic number with ??? m conf->timeStep * mconf->attackSpeed / 2.0;

  const auto target_count = targets_.getTargetCount();
  targetDepo.assign(static_cast<std::size_t>(target_count), TargetControl{});

  for (auto& target : targetDepo) {  // initialise each tgt state
    target.state = ACTIVE;
  }

  stats.total = target_count;
  stats.active = target_count;
  stats.solverName = solver_->name();
  stats.ammoName = ammo.name;
  mstate = std::make_unique<mission::Idle>();
}

MissionProcessor::~MissionProcessor()
{
  j_out["totalSteps"] = stats.steps;

  std::ofstream jf_out(simulationPath);
  if (jf_out.is_open()) {
    jf_out << j_out.dump(2);  // 2 spaces => tab
  }
}

auto MissionProcessor::getInstantAimPoint(drone::DroneTelemetry& telemetry) -> pointmath::Point
{
  dto::BallisticResult ballResult = solver_->solve(telemetry.z, telemetry.speed, ammo);  // TODO check exception in Analytical solver

  return pointmath::Point{telemetry.x, telemetry.y} + pointmath::Point{telemetry.vx, telemetry.vy} * ballResult.hDist;
};

/* // Записати дані кроку у вихідн. JSON файл */ /* TODO
Формат той самий, що в попередніх ДЗ: масив steps з полями
position, direction, state, targetIndex, dropPoint, aimPoint, predictedTarget.
один запис на крок планування;
у поле state пишеться поточний DroneMode з телеметрії.
Додаткове поле для врахування нерівномірності кроків - timeSecSinceStart.
Якщо цого поля не буде, чекер буде рахувати кроки рівномірними.
 */
auto MissionProcessor::pushStepToJSON(drone::DroneTelemetry& telemetry) -> void
{
  Point aimPoint = getInstantAimPoint(telemetry);  // mission.getAmmoHDist());
  json step;
  step["timeSecSinceStart"] = static_cast<float>(telemetry.t_ms) / 1000.0;  // крок х-дрона у-дрона кут-дрона стан-дрона ціль№
  step["position"] = {{"x", telemetry.x}, {"y", telemetry.y}};
  step["direction"] = telemetry.dir;
  step["state"] = drone_.stateToStr(telemetry.state);
  step["targetIndex"] = mctx.currentTgtTag;

  step["aimPoint"] = {{"x", aimPoint.x}, {"y", aimPoint.y}};

  if (mctx.currentTgtTag >= 0) {
    step["dropPoint"] = {{"x", mctx.firePoint.x}, {"y", mctx.firePoint.y}};

    step["predictedTarget"] = {{"x", mctx.tgtLeadPos.x}, {"y", mctx.tgtLeadPos.y}};
  }
  else {                                // no target, the fields not defined
    step["dropPoint"] = nullptr;        // {{"x", 0}, {"y", 0}};        // to have same structure //nullptr;  //
    step["predictedTarget"] = nullptr;  // {{"x", 0}, {"y", 0}};  // to have same structure //nullptr;
  }
  j_out["steps"].push_back(step);

  DEBUG(TimeTracker::getInstance().getElapsed()
        << " X: " << telemetry.x << " Y: " << telemetry.y << " Dir " << telemetry.dir << " " << drone_.stateToStr(telemetry.state) << " T#"
        << mctx.currentTgtTag << " Aim " << aimPoint << " FP " << mctx.firePoint << " TLP " << mctx.tgtLeadPos << " " << mstate->name());
}

}  // namespace core
