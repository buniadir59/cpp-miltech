#include "mission/MissionProcessor.hpp"
#include "config/TimeTracker.hpp"
#include "dto/BallisticResult.hpp"
//#include "dto/Target.hpp"
//#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "math/point_math.hpp"
#include "link/drone_link.h"

#define DBG_MODE
#ifdef DBG_MODE                // #endif
#include "config/defines.hpp"  //for DEBUG
#endif

//#include <memory>
#include <nlohmann/json.hpp>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <thread>

using json = nlohmann::json;

namespace {
/* 
Другий важливий нюанс: у PKT_TARGET з ТЗ є тільки id, x, y, без швидкості. А  стара логіка 
lead point потребує Target.velocity -> треба зробити маленький TargetTracker, 
який для кожної цілі зберігає попередню позицію й час, і рахує швидкість кінцевою різницею:
    velocity = (currentPosition - previousPosition) / dt;
Час краще брати з останньої Telemetry.t_ms, бо це час симуляції чекера
*/
  /* //TODO 
    є нюанс: у HW11 CONTROL.accel = 0 означає “тримати швидкість”, 
  тому для MOVING після розгону, можливо, краще слати 0, а не 1. 
  Тобто на практиці ControlMapper має дивитися ще й на поточну telemetry.speed. */
auto mapCommand(const dto::DroneCommand& cmd, 
                const dto::MissionConfig& conf) -> dlink::Control
{
  dlink::Control out{};

  switch (cmd.state) {
    case dto::ACCELERATING:
    case dto::MOVING:
      out.accel = 1.0F;
      break;

    case dto::DECELERATING:
    case dto::STOPPED:
      out.accel = -1.0F;
      break;

    case dto::TURNING:
      out.accel = 0.0F;
      break;
  }

  out.turnRate = static_cast<float>(
      std::clamp(cmd.angleSpeed / conf.maxAngularSpeedRadPerS, -1.0, 1.0));

  return out;
}

auto stateToStr(unsigned int state_num, bool& fired) -> const char*
{
  if (fired) {
    fired = false;
    return "ATTACK";
  }
  switch (state_num) {
    case dto::STOPPED:
      return "STOPPED";
    case dto::ACCELERATING:
      return "ACCELERATING";
    case dto::DECELERATING:
      return "DECELERATING";
    case dto::MOVING:
      return "MOVING";
    case dto::TURNING:
      return "TURNING";
  }
  return "UNKNWN";
}

}  // namespace
namespace core {

auto MissionProcessor::run() noexcept -> void
{
  try {
    init();
    threadReady.store(true);

    while (!threadStart.load() && !threadStopRequested.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    TimeTracker& tt = TimeTracker::getInstance();

    while (!threadStopRequested.load() && hasNext()) {
      if (!step()) {
        break;
      }
      auto wakeup = tt.nextWakeup(mctx.mconf.timeStep);
      std::this_thread::sleep_until(wakeup);
    }
  }
  catch (const std::exception& error) {
    std::cerr << "Processor thread error: " << error.what() << '\n';
    threadStopRequested.store(true);
    failed.store(true);
  }
  catch (...) {
    std::cerr << "Processor thread unknown error\n";
    threadStopRequested.store(true);
    failed.store(true);
  }
}

// return false if #steps > max
bool MissionProcessor::step()
{
  if (stats.steps >= kMaxSteps) {  // simulation is over!
    return false;
  }

  updateTargets();  // get new positions and other parames & unreachable => active
 //TODO  mctx.telemetry = drone_.getTelemetry();
  if (mctx.telemetry.speed > 0.0) {
    dto::BallisticResult ballResult = solver_->solve(mctx.mconf.kAltitude, mctx.telemetry.speed, ammo);

    mctx.instantAmmoFFTime = ballResult.ffTime;
    mctx.instantAmmoFFDist = ballResult.hDist;
  }
  else {  // no valid solution
    mctx.instantAmmoFFTime = 0.0;
    mctx.instantAmmoFFDist = 0.0;
  }
  mctx._updateSpeedDependentCtx();
  auto next = mstate->execute(mctx);
  if (next) {
    mstate = std::move(next);
  }
  //TODO drone_.sendCommand(mctx.cmd); 
  pushStepToJSON();

  ++stats.steps;
  return true;
}


// gets new targets position and velocity, skipping destroyed
// for attacked targets checks if the ammo hit the ground,
// if yes, verifies result of attacked - hit or miss
// restores status active for unreachable targets
auto MissionProcessor::updateTargets() -> void
{
  for (std::size_t i = 0; i < targetDepo.size(); ++i) {
    if (targetDepo[i].state == DESTROYED)
      continue;

 //TODO   targetDepo[i].now = targets_.getTarget(i);
    targetDepo[i].update();

    switch (targetDepo[i].state) {
      case ACTIVE:
        break;

      case ATTACKED:
        if ((targetDepo[i].hitTime - TimeTracker::getInstance().getElapsed()) < mctx.mconf.timeStep / 2.0) {
          stats.firedCount++;
          double dist = pointmath::getLength(targetDepo[i].now.position - targetDepo[i].hitCoord);
          if (dist <= mctx.mconf.hitRadius) {
            targetDepo[i].state = core::DESTROYED;
            stats.destroyed++;  // destroyed is final state
          }
          else {
            targetDepo[i].state = core::ACTIVE;
          }

          LOG(TimeTracker::getInstance().getElapsed()  // report on fire
              << " T#" << i << " Res= " << targetDepo[i].targetStateToStr() << " at_dist " << dist << " hitXY: " << targetDepo[i].hitCoord.x
              << ' ' << targetDepo[i].hitCoord.y << " TPos " << targetDepo[i].now.position.x << ' ' << targetDepo[i].now.position.y
              << " TSpeed " << targetDepo[i].speed);
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

auto MissionProcessor::updateBasicAmmoRes() -> void
{
  dto::BallisticResult ballResult = solver_->solve(mctx.mconf.kAltitude, mctx.mconf.attackSpeed, ammo);
  mctx.ammoBaseFFTime = ballResult.ffTime;
  mctx.ammoBaseHDist = ballResult.hDist;
}

auto MissionProcessor::init() -> void
{
  j_out["steps"] = json::array();
  updateBasicAmmoRes();
  // initialise  tgts depo
  const auto target_count = 5;  //TODO targets_.getTargetCount();
  targetDepo.assign(static_cast<std::size_t>(target_count), TargetControl{});
  for (auto& target : targetDepo) {
    target.state = ACTIVE;
  }

  stats.total =  target_count;
  stats.active = target_count;
  stats.solverName = solver_->name();
  stats.ammoName = ammo.name;
  mstate = std::make_unique<mission::Idle>();
#ifdef DBG_MODE
  DEBUG("Time TTime T# X: Y: FpX FpY D2FP T2FP DrDir° A2T° AimX AimY TlpX TlpY HiDist cmd.As cmd.St res DrSt MiSt");
#endif
}

MissionProcessor::~MissionProcessor()
{
  j_out["totalSteps"] = stats.steps;

  std::ofstream jf_out(simulationPath);
  if (jf_out.is_open()) {
    jf_out << j_out.dump(2);  // 2 spaces => tab
  }
}

auto MissionProcessor::getSimulationStatistics() -> const dto::SimStatistics&
{
  stats.underAttack = std::count_if(targetDepo.begin(), targetDepo.end(), [](const TargetControl& tgt) { return tgt.state == ATTACKED; });
  stats.destroyed = std::count_if(targetDepo.begin(), targetDepo.end(), [](const TargetControl& tgt) { return tgt.state == DESTROYED; });
  stats.total = targetDepo.size();
  stats.active = stats.total - stats.destroyed - stats.underAttack;
  return stats;
}

/* // Записати дані кроку у вихідн. JSON файл
Формат той самий, що в попередніх ДЗ: масив steps з полями
position, direction, state, targetIndex, dropPoint, aimPoint, predictedTarget.
один запис на крок планування;
у поле state пишеться поточний DroneMode з телеметрії.
Додаткове поле для врахування нерівномірності кроків - timeSecSinceStart.
Якщо цого поля не буде, чекер буде рахувати кроки рівномірними.
 */
auto MissionProcessor::pushStepToJSON() -> void
{
  pointmath::Point aimPoint = mctx.instantAimPoint;  // mission.getAmmoHDist());
  json step;
  step["timeSecSinceStart"] = mctx.telemetry.timeSecSinceStart;  // крок х-дрона у-дрона кут-дрона стан-дрона ціль№
  step["position"] = {{"x", mctx.telemetry.x}, {"y", mctx.telemetry.y}};
  step["direction"] = mctx.telemetry.dir;
  step["state"] = stateToStr(mctx.telemetry.state, mctx.fired);
  step["targetIndex"] = mctx.currentTgtTag;

  step["aimPoint"] = {{"x", aimPoint.x}, {"y", aimPoint.y}};

  if (mctx.currentTgtTag >= 0) {
    step["dropPoint"] = {{"x", mctx.firePoint.x}, {"y", mctx.firePoint.y}};

    step["predictedTarget"] = {{"x", mctx.tgtLeadPos.x}, {"y", mctx.tgtLeadPos.y}};
  }
  else {  // no target, the fields not defined
    step["dropPoint"] = nullptr;
    step["predictedTarget"] = nullptr;
  }
  j_out["steps"].push_back(step);

// DEBU G("Time TTime T# X: Y: FpX FpY D2FP T2FP DrDir A2T AimX AimY TlpX TlpY HiDist cmd.As cmd.St res DrSt MiSt");
#ifdef DBG_MODE
  DEBUG(TimeTracker::getInstance().getElapsed()  // full step report
        << " " << mctx.telemetry.timeSecSinceStart << " " << mctx.currentTgtTag << " " << mctx.telemetry.x << " " << mctx.telemetry.y << " "
        << mctx.firePoint.x << " " << mctx.firePoint.y << " " << mctx.dist2fp << " " << mctx.time2fp << " "
        << anglemath::rad2Grad(mctx.telemetry.dir) << " " << anglemath::rad2Grad(mctx.angle_to_tgt) << " " << aimPoint.x << " "
        << aimPoint.y << " " << mctx.tgtLeadPos.x << " " << mctx.tgtLeadPos.y << " " << mctx.hit_dist << " " << mctx.cmd.angleSpeed << " "
        << stateToStr(mctx.cmd.state, mctx.fired) << " " << mctx.res_code << " " << stateToStr(mctx.telemetry.state, mctx.fired) << " "
        << mstate->name());
#endif
}

}  // namespace core
