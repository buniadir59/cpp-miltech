#include "drone.hpp"
#include "point_math.hpp"
#include "ammo.hpp"
#include "simulation.hpp"

#include "json.hpp"

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
const char* const kAmmosPath = "homework_03/data/ammo.json";
const char* const kInputPath = "homework_03/data/config.json";
const char* const kTargetsPath = "homework_03/data/targets.json";
const char* const kSimulationPath = "homework_03/data/simulation.json";

// ## max number of simulation steps if any target not hit
constexpr int kMaxSteps = 10000;

// **** helpers: read  input data from files write simulation output to file

auto readTargetsJSON(const char* file_path, sim::Simulation& sim)
{
  std::ifstream tgts_json(file_path);
  if (!tgts_json.is_open()) {
    std::cerr << "Unable to open: " << file_path << '\n';
    return 1;
  }

  try {
    json tgts;
    tgts_json >> tgts;

    size_t tgtsCount = tgts["targetCount"];
    size_t timeSteps = tgts["timeSteps"];
   
    auto coords = new Point*[tgtsCount]; // Point** 

    for (size_t i = 0; i < tgtsCount; ++i) {
      coords[i] = new Point[timeSteps];

      for (size_t j = 0; j < timeSteps; ++j) {
        coords[i][j].x = tgts["targets"][i]["positions"][j]["x"];
        coords[i][j].y = tgts["targets"][i]["positions"][j]["y"];
      }
    }

    sim.nTgts = tgtsCount;
    sim.nTargetSteps = timeSteps;
    sim.tgtTracks = coords;
  }
  catch (const std::exception& error) {
    std::cerr << "Invalid or incomplete data in " << file_path << '\n';
    return 1;
  }

  return 0;
}

auto readJSONInput(const char* file_path, sim::SimConfig& init_sim, drone::DroneConfig& init_dr)
{
  std::ifstream input_json(file_path);

  if (!input_json.is_open()) {
    std::cerr << "Unable to open: " << file_path << '\n';
    return 1;
  }

  try {
    json jsn;
    input_json >> jsn;

    init_dr.position = {jsn["drone"]["position"]["x"], jsn["drone"]["position"]["y"]};
    init_dr.altitude = jsn["drone"]["altitude"];
    init_dr.initial_direction = jsn["drone"]["initialDirection"];
    init_dr.attack_speed = jsn["drone"]["attackSpeed"];
    init_dr.acceleration_path = jsn["drone"]["accelerationPath"];
    init_dr.angular_speed = jsn["drone"]["angularSpeed"];
    init_dr.turn_threshold = jsn["drone"]["turnThreshold"];

    init_sim.hit_rad = jsn["simulation"]["hitRadius"];
    init_sim.time_step = jsn["simulation"]["timeStep"];

    init_sim.tgt_time_step = jsn["targetArrayTimeStep"];

    std::strncpy(init_dr.ammo_name, jsn["ammo"].get<std::string>().c_str(), sizeof(init_dr.ammo_name) - 1);
    init_dr.ammo_name[sizeof(init_dr.ammo_name) - 1] = '\0';
  }
  catch (const std::exception& error) {
    std::cerr << "Invalid or incomplete data in " << file_path << '\n';
    return 1;
  }

  return 0;
}

auto readAmmosJSON(const char* file_path, sim::Simulation& sim) -> int
{
  std::ifstream ammos_json(file_path);
  if (!ammos_json.is_open()) {
    std::cerr << "Unable to open: " << file_path << '\n';
    return 1;
  }

  try {
    json ammos;
    ammos_json >> ammos;

    size_t nAmmos = ammos.size();

    sim.ammoTable = new ammo::Ammo[nAmmos];
    sim.nAmmos = nAmmos;

    for (size_t i = 0; i < nAmmos; ++i) {
      std::strncpy(sim.ammoTable[i].name, ammos[i]["name"].get<std::string>().c_str(), sizeof(sim.ammoTable[i].name) - 1);
      sim.ammoTable[i].name[sizeof(sim.ammoTable[i].name) - 1] = '\0';
      sim.ammoTable[i].mass = ammos[i]["mass"];
      sim.ammoTable[i].drag = ammos[i]["drag"];
      sim.ammoTable[i].lift = ammos[i]["lift"];
    }
    return 0;
  }
  catch (const std::exception& error) {
    std::cerr << "Invalid or incomplete data in " << file_path << '\n';
  }

  return 1;
}

// Записати дані кроку у вихідн. JSON файл
auto pushStepToJSON(json& out, const drone::SimStep& sim_step) -> void
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

}  // namespace

// ################################################################################
auto main() -> int
{
  try {
    json j_out;
    j_out["steps"] = json::array();
    sim::SimConfig sim_init{};
    drone::DroneConfig dr_init{};

    if (0 != readJSONInput(kInputPath, sim_init, dr_init))
      return 1;

    sim::Simulation sim{sim_init};
    if ((0 != readAmmosJSON(kAmmosPath, sim)) || (0 != readTargetsJSON(kTargetsPath, sim))) {
      sim.freeMemory();
      return 1;
    }

    dr_init.number_of_targets = sim.nTgts;
    dr_init.ammo = ammo::findAmmoByName(sim.ammoTable, sim.nAmmos, dr_init.ammo_name);
    if (dr_init.ammo == nullptr) {
      std::cerr << "Unknown ammo in config.json:" << dr_init.ammo_name << '\n';
      sim.freeMemory();
      return 1;
    };

    drone::Drone dr{dr_init};

    std::cout << std::fixed << std::setprecision(1);

    DEBUG("\tDr:" << dr.coord << " Dir" << dr.dirRad << ' ' << dr.droneStateToStr());
    DEBUG("\tAltitude,m: " << dr.alt << "\n\tAttack_Speed,m/s: " << dr.attSpeed << "\n\tAcceleration_path,m: " << dr.accPath
                           << "\n\tAngular_Speed,rad: " << dr.angSpeed << "\n\tTurn_Threshold,rad: " << dr.turnThrld
                           << "\n\tAmmo: " << dr.ammo->name << ", m=" << dr.ammo->mass << ", d=" << dr.ammo->drag << ", l=" << dr.ammo->lift
                           << "\n\tHit_Radius,m:#" << sim.kHitRad << " acc,m/ss: " << dr.kAcceleration);

    int stepCurrent = 0;       // step, incremented through simulation until maximum
    double timeCurrent = 0.0;  // current time
    sim.initializeTgtPositions(dr);
    dr.startNewMission(sim.timeStep);  // calculate first ballistic solutions, skip results

    for (size_t i = 0; i < dr.nTargets; ++i) {
      DEBUG("#T " << dr.tgts[i]);  // printAllTargets
    }      

    dr.updateSimStep(sim.simStep);
    pushStepToJSON(j_out, sim.simStep);  // save step 0 data

     while (!dr.isMissionCompleted()) {
      // move simulation time forward and check if we are not over max steps
      ++stepCurrent;
      timeCurrent += sim.timeStep;
      if (stepCurrent > kMaxSteps) {  // simulation is over!
        LOG("No hit, simulation time is over!");
        break;
      };

      // move targets and inform drone of their current positions, move drone according to its state, direction and speed
      sim.moveTargets(timeCurrent, dr);
      dr.moveDrone(sim.timeStep);

      // if we are on the way to target, continue mission
      if (dr.isOnMission()) {
        DEBUG("\tT#" << dr.mission.tgtTag << ' ' << dr.tgts[dr.mission.tgtTag]);
        DEBUG("\tOn" << dr.mission);

        // analyze drone position on the drop path and change its state accordingly
        if (dr.continueMission(sim.timeStep)) {  // fired, check  hit or miss
          Point hit_coord;
          double time_of_hit = timeCurrent + dr.getHitCoordAndAmmoFlyTime(hit_coord);
          // check if hit expected
          pointmath::Point tgt_pos_at_hit = sim.getTgtPositionAt(dr.mission.tgtTag, time_of_hit);
          double hit_dist = pointmath::getLength(tgt_pos_at_hit - hit_coord);

          LOG("\tFired at: step#" << stepCurrent << "(" << timeCurrent << "s). Target coordinates at hit " << tgt_pos_at_hit
                                  << ", distance: " << hit_dist << 'm');

          if (sim.kHitRad > hit_dist) {
            LOG("Hit!");
            dr.completeMission();
          }
          else {
            LOG("Missed!");
            dr.breakMission();
          };
        }
      }

      // if we are not on the mission (mission failed, or cant be achieved anymore), try to start new one
      if (!dr.isOnMission()) {
        dr.mission.tgtTag = dr.startNewMission(sim.timeStep);

        if (dr.isOnMission()) {
          for (size_t i = 0; i < dr.nTargets; ++i)
            DEBUG("#T " << dr.tgts[i]);  // printAllTargets
          LOG("\tStarting" << dr.mission);
        }
      }

      DEBUG("#" << stepCurrent << " (" << timeCurrent << "s) Dr:" << dr.coord << " Dir" << dr.dirRad << ' ' << dr.droneStateToStr()
                << " v=" << dr.speed << " err=" << dr.errcode);

      dr.updateSimStep(sim.simStep);
      pushStepToJSON(j_out, sim.simStep);

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
    sim.freeMemory();
    return result;

  }  // eo try

  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }
}
