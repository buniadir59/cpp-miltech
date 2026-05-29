#include "drone.hpp"
#include "dto/MissionConfig.hpp"
#include "math/point_math.hpp"
#include "config/FileConfigLoader.hpp"
#include "providers/JsonTargetProvider.hpp"
#include "ManualSimulationClock.hpp"

#include <nlohmann/json.hpp>

#include <cstddef>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <cstring>

using json = nlohmann::json;
using Point = pointmath::Point;

/* **** defines and contants **** */

#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl;
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl;
#else
#define DEBUG(msg)
#endif

/* main.cpp - read input files, create objects, run simulation loop, manage output */

namespace {
//const char* const kAmmosPath = "homework_03/data/ammo.json";
const char* const kInputPath = "homework_07/data";
//const char* const kTargetsPath = "homework_07/data/targets.json";
const char* const kSimulationPath = "simulation.json";

// ## max number of simulation steps if any target not hit
constexpr int kMaxSteps = 100; //TODO !!! 10000;

// **** helpers: read  input data from files write simulation output to file

// Записати дані кроку у вихідн. JSON файл
auto pushStepToJSON(json& out, const dto::SimStep& sim_step) -> void
{
  json step;  // крок х-дрона у-дрона кут-дрона стан-дрона ціль№
  step["position"] = {{"x", sim_step.pos.x}, {"y", sim_step.pos.y}};
  step["direction"] = sim_step.direction;
  step["state"] = sim_step.state;
  step["targetIndex"] = sim_step.targetIdx;
  if (sim_step.targetIdx >= 0) {
    step["dropPoint"] = {{"x", sim_step.dropPoint.x}, {"y", sim_step.dropPoint.y}};
    if (sim_step.state == drone::MOVING) {
      step["aimPoint"] = {{"x", sim_step.aimPoint.x}, {"y", sim_step.aimPoint.y}};
    }
    else {
      step["aimPoint"] = nullptr;
    }
    step["predictedTarget"] = {{"x", sim_step.predictedTarget.x}, {"y", sim_step.predictedTarget.y}};
  }
  else {  // no target, the fields not defined
    step["dropPoint"] = nullptr;
    step["aimPoint"] = nullptr;
    step["predictedTarget"] = nullptr;
  }
  out["steps"].push_back(step);
}

std::ostream& operator<<(std::ostream& os, const dto::Mission& m)
{
  if (m.tgtTag < 0)
    return os << "=>No target";

  return os << "=>T#" << m.tgtTag << " tmr=" << m.timer << " TA" << m.destAngle << " Dest" << m.destPoint << ' ' << m.missionStateToStr()
            << " TLP" << m.tgt_lead_pos;
}

}  // namespace

// ################################################################################
auto main() -> int
{
  try {
    json j_out;
    j_out["steps"] = json::array();
    

    FileConfigLoader confloader;
    if (!confloader.load(kInputPath)) {
      throw std::runtime_error("Error loading configuration");
    };
    const dto::MissionConfig& mconf = confloader.getConfig();

    ManualSimulationClock simClock;
    JsonTargetProvider tgtProvider(kInputPath, mconf.tgt_time_step, &simClock);
   
    drone::Drone dr{mconf, &tgtProvider};
    dr.ammo = &confloader.getAmmoParams();

    std::cout << std::fixed << std::setprecision(1);

    DEBUG("\tDr:" << dr.coord << " Dir" << dr.dirRad << ' ' << dr.droneStateToStr());
    DEBUG("\tAltitude,m: " << dr.alt << "\n\tAttack_Speed,m/s: " << dr.attSpeed << "\n\tAcceleration_path,m: " << dr.accPath
                           << "\n\tAngular_Speed,rad: " << dr.angSpeed << "\n\tTurn_Threshold,rad: " << dr.turnThrld
                           << "\n\tAmmo: " << dr.ammo->name << ", m=" << dr.ammo->mass << ", d=" << dr.ammo->drag << ", l=" << dr.ammo->lift
                           << "\n\tHit_Radius,m:#" << mconf.hit_rad << " acc,m/ss: " << dr.kAcceleration);
    
    
    dto::SimStep simStep{}; //TODO ?
    int stepCurrent = 0;       // step, incremented through simulation until maximum
    //double timeCurrent = 0.0;  // current time
  //  sim.initializeTgtPositions(dr);
    dr.startNewMission(mconf.time_step);  // calculate first ballistic solutions, skip results

    for (size_t i = 0; i < dr.nTargets; ++i) {
      DEBUG("#T " << dr.tgts[i]);  // printAllTargets
    }      

    dr.updateSimStep(simStep);
    pushStepToJSON(j_out, simStep);  // save step 0 data

     while (!dr.isMissionCompleted()) {
      // move simulation time forward and check if we are not over max steps
      ++stepCurrent;
      simClock.advance(mconf.time_step);  //   timeCurrent += sim.timeStep;
      if (stepCurrent > kMaxSteps) {  // simulation is over!
        LOG("No hit, simulation time is over!");
        break;
      };

      // move targets and inform drone of their current positions, move drone according to its state, direction and speed
  //    sim.moveTargets(timeCurrent, dr);
      dr.moveDrone(mconf.time_step);

      // if we are on the way to target, continue mission
      if (dr.isOnMission()) {
        DEBUG("\tT#" << dr.mission.tgtTag << ' ' << dr.tgts[dr.mission.tgtTag]);
        DEBUG("\tOn" << dr.mission);

        // analyze drone position on the drop path and change its state accordingly
        if (dr.continueMission(mconf.time_step)) {  // fired, check  hit or miss
          Point hit_coord;
          const double timeOfFire = simClock.nowS();
          const double time_of_hit = timeOfFire + dr.getHitCoordAndAmmoFlyTime(hit_coord);
          // check if hit expected TODO
        //  pointmath::Point tgt_pos_at_hit = sim.getTgtPositionAt(dr.mission.tgtTag, time_of_hit);
        //  double hit_dist = pointmath::getLength(tgt_pos_at_hit - hit_coord);
          LOG("\tFired at: step#" << stepCurrent << "(" << timeOfFire << "s).");
          dr.breakMission();
          // LOG("\tFired at: step#" << stepCurrent << "(" << timeOfFire << "s). Target coordinates at hit " << tgt_pos_at_hit
          //                         << ", distance: " << hit_dist << 'm');

          //TODO check if Hit // if (sim.kHitRad > hit_dist) {
          //   LOG("Hit!");
          //   dr.completeMission();
          // }
          // else {
          //   LOG("Missed!");
          //   dr.breakMission();
          // };
        }
      }

      // if we are not on the mission (mission failed, or cant be achieved anymore), try to start new one
      if (!dr.isOnMission()) {
        dr.mission.tgtTag = dr.startNewMission(mconf.time_step);

        if (dr.isOnMission()) {
          for (size_t i = 0; i < dr.nTargets; ++i)
            DEBUG("#T " << dr.tgts[i]);  // printAllTargets
          LOG("\tStarting" << dr.mission);
        }
      }

      DEBUG("#" << stepCurrent << " (" << simClock.nowS() << "s) Dr:" << dr.coord << " Dir" << dr.dirRad << ' ' << dr.droneStateToStr()
                << " v=" << dr.speed << " err=" << dr.errcode);

      dr.updateSimStep(simStep);
      pushStepToJSON(j_out, simStep);

    } //eo while

    int result = dr.isMissionCompleted() ? 0 : 1;

    j_out["totalSteps"] = stepCurrent + 1;

    DEBUG(" countMaxRecalc" << dr.countMaxRecalc);

    std::ofstream jf_out(kSimulationPath);
    if (jf_out.is_open()) {
      jf_out << j_out.dump(2);  // 2 spaces => tab
    }
    else {
      std::cerr << "Unable to open output file: " << kSimulationPath << '\n';
      result = 1;
    }

    dr.freeMemory();
    return result;

  }  // eo try

  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }
}
