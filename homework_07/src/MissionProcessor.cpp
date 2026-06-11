#include "defines.hpp"
#include "dto/SimStatistics.hpp"
#include "dto/Target.hpp"
#include "TargetControl.hpp"
#include "MissionProcessor.hpp"
#include "DroneControl.hpp"
//#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "math/angle_math.hpp"
#include "math/point_math.hpp"

#include <cmath>
#include <nlohmann/json.hpp>

#include <stdexcept>
#include <fstream>
#include <optional>

using json = nlohmann::json;

using Point = pointmath::Point;
using AngleRad = anglemath::AngleRad;

namespace core {

auto MissionProcessor::init(const char* configSource) -> void {
    if (!loader_->load(configSource)) {
      throw std::runtime_error("Error loading configuration");
    };
    
    j_out["steps"] = json::array();

    mconf = &(loader_->getConfig());
    ammo = loader_->getAmmoParams();

    drone.emplace(*mconf);
    mission.init(mconf->time_step, &*drone, ammo);
   
    target_count_ = targets_->getTargetCount();

    for (int i = 0; i < target_count_; ++i) {
      targetDepo[i].state = ACTIVE; 
    }

  currentTgtTag = 0;
  stepCurrent = 0;

  updateTargets();

  pushStepToJSON();

/*     int count = START_FROM_TGT;
    while (mission.startNewMission(targetDepo[START_FROM_TGT]) == false) {  //@init
      if (++count > target_count_) {
        throw std::runtime_error("ERR: no possible missions found");
      }
    };    //@init 

    pushStepToJSON(); // save step 0 data*/
}

auto MissionProcessor::checkFireResult(TargetControl& tgt) -> bool {
  if (std::abs(clock->nowS() - tgt.hitTime) <= mconf->time_step / 2.0) {
    double dist = pointmath::getLength(tgt.now.position - tgt.hitCoord);
    LOG("Hit at dist=" << dist << " hit_time=" << tgt.hitTime);
    if (dist <= mconf->hit_rad) {
      tgt.state = core::DESTROYED;
    } else {
      tgt.state = core::ACTIVE;
    }
    return true;
  }
  return false;
}

auto MissionProcessor::getSimulationStatistics() -> dto::SimStatistics& {
  int countAct=0, countDestrd=0, countUnderAtt=0;
   for (const auto& tgt: targetDepo) {
    switch(tgt.state) {
      case ACTIVE:
        ++countAct;
        break;
      case ATTACKED:
      ++countUnderAtt;
      break;
      case DESTROYED:
        ++countDestrd;
        break;
      default:
        break;
    }
    stats.active = countAct;
    stats.destroyed = countDestrd;
    stats.underAttack = countUnderAtt;
    stats.total = countAct + countUnderAtt + countDestrd;
    stats.timeElapsed =clock->nowS();
   }
   return stats;
}

// gets new targets position and velocity 
auto MissionProcessor::updateTargets() -> void {
    for (auto i = 0; i < target_count_; ++i) {
      if (targetDepo[i].state == DESTROYED) continue;

      targetDepo[i].now = targets_->getTarget(i); 
      targetDepo[i].update(); 

      switch (targetDepo[i].state) {
        case ACTIVE:
          break;

        case ATTACKED:
          if (checkFireResult(targetDepo[i])) {
            LOG(i << ": time=" << clock->nowS()
                  << " Result of attack=" << targetDepo[i].targetStateToStr());
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
//return false if #steps > max
bool MissionProcessor::step() {
    ++stepCurrent;
     
    if (stepCurrent > defines::kMaxSteps) {  // simulation is over!
      return false;   
    }

    updateTargets(); //unreachable => active

    drone->move(mconf->time_step);  

    if (mission.isOnMission()) {
        // analyze dron position on the drop path and change its state accordingly
        if (mission.continueMission()) {  // fired 
          fire();
          mission.breakMission(); //@fired
        }
    }

    if (!mission.isOnMission()) {
      while (hasNext() ) { 
      
        if (currentTgtTag >= 0) { //active target found @step
            LOG("Try T#" << currentTgtTag << ' ' << targetDepo[currentTgtTag].targetStateToStr());
          if (mission.startNewMission(targetDepo[currentTgtTag]) == true) { //go on mission
            break;  
          } else {
            LOG("Skipped T#" << currentTgtTag << ' ' << targetDepo[currentTgtTag].targetStateToStr());//targetDepo[currentTgtTag].state = UNREACHABLE; 
          }
        } else { //no available targets, wait 
          LOG("@Wait!");
          break;
        }
      }
    } //eo is on mission

    pushStepToJSON(); 

    DEBUG(stepCurrent <<" step to T#" << currentTgtTag << '@' << drone->droneStateToStr() 
        << "@Dr@" << drone->coord 
        << "@Dir" << drone->dirRad << "@v=" << drone->speed 
        );
    return true;
}
/* 
auto MissionProcessor::getNextTarget() const -> int {
  for (int i = currentTgtTag; i < target_count_; ++i) {
    if (targetDepo[i].state == ACTIVE) {

      return i;
    }
  }

  return -1;
}
 */
auto MissionProcessor::getNextTarget() const -> int {
  int start_idx = currentTgtTag;

  if (start_idx < 0) {
    start_idx = 0;
  }

  for (int i = start_idx; i < target_count_; ++i) {
    if (targetDepo[i].state == ACTIVE) {
      return i;
    }
  }

  return -1;
}

// find and  set next Active target (current tgt tag)
// return false if all destroyed, true if active or under attack or unreachable tgts exist
auto MissionProcessor::identifyNextTarget()-> bool {
      // if we are not on the mission (mission failed, or cant be achieved anymore), try to start new one
    int nextIdx = getNextTarget();
    if (nextIdx < 0) { //end of round over targets
       reset(); //start over 
       nextIdx = getNextTarget();
       if (nextIdx < 0) {  //no active targets, check if any of the targets are in Attacked state
            bool some_unavailable = false;
            for (auto i = 0; i < target_count_; ++i) {
              if(targetDepo[i].state == ATTACKED || targetDepo[i].state == UNREACHABLE) {
                currentTgtTag = nextIdx;  
                some_unavailable = true;
                break;
              }
            }
            return some_unavailable; //wait for fire result or end simulation 
       }
    }

    //set found active target as next
    currentTgtTag = nextIdx;   
    return true;
}


 //Перевірити, чи є ще необроблені цілі 
 // NB! for simulation will always return true
auto MissionProcessor::hasNext() -> bool { 
  if (!mission.isOnMission()) { 
    return (targets_->getTargetCount() != 0) &&  (identifyNextTarget());
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



auto MissionProcessor::fire() -> void {
      targetDepo[currentTgtTag].state = core::ATTACKED;     
      targetDepo[currentTgtTag].hitCoord = drone->getAimPoint(mission.ammoHorizDist); 
      targetDepo[currentTgtTag].hitTime = clock->nowS() + mission.ammoFlyTime; 
      LOG("@at FP: Fired!@"<< currentTgtTag <<"time=" << clock->nowS() << "@hitCoord@" << targetDepo[currentTgtTag].hitCoord );
}

} //eo namespace core


