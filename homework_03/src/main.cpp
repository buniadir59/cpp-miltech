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
  const char* kAmmosPath = "homework_03/data/ammo.json";
  const char* kInputPath = "homework_03/data/config.json";
  const char* kTargetsPath = "homework_03/data/targets.json";
  const char* kSimulationPath = "homework_03/data/simulation.json"; 

  // ## max number of simulation steps if any target not hit
  constexpr int kMaxSteps = 10000; 

// **** helpers: read  input data from files write simulation output to file

  auto readTargetsJSON(const std::string& file_path, sim::SimConfig& init_sim, drone::DroneConfig& init_dr) { 
	  std::ifstream tgts_json (file_path);
	  if (!tgts_json.is_open()) {
		  throw std::runtime_error("Unable to open " + file_path);
	  }

    try {
      json tgts;
      tgts_json >> tgts;

      size_t tgtsCount = tgts["targetCount"];
      size_t timeSteps = tgts["timeSteps"];
      Point** coords = new Point* [tgtsCount];

      for (size_t i = 0; i < tgtsCount; ++i) {
          coords[i] = new Point[timeSteps]; 

          for (size_t j = 0; j < timeSteps; ++j){
            coords[i][j].x = tgts["targets"][i]["positions"][j]["x"];
            coords[i][j].y = tgts["targets"][i]["positions"][j]["y"];
          }
      }
      
      init_sim.number_of_targets = tgtsCount; 
      init_dr.number_of_targets = tgtsCount; 
      init_sim.target_steps = timeSteps;   
      init_sim.target_tracks = coords;

    } catch (const std::exception& error) {
        std::cerr << "Invalid or incomplete data in " << file_path << '\n';
      return 1;
    }  

    return 0;
  } 

  auto readJSONInput(const std::string& file_path, sim::SimConfig& init_sim, drone::DroneConfig& init_dr) { 
    std::ifstream input_json(file_path);

    if (!input_json.is_open()) {
      throw std::runtime_error("Unable to open input file: " + file_path);
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
        
      init_dr.hit_radius = jsn["simulation"]["hitRadius"];
      init_sim.time_step = jsn["simulation"]["timeStep"];

      init_sim.tgt_time_step = jsn["targetArrayTimeStep"];

      std::strncpy(init_dr.ammo_name, jsn["ammo"].get<std::string>().c_str(), sizeof(init_dr.ammo_name) - 1);
      init_dr.ammo_name[sizeof(init_dr.ammo_name) - 1] = '\0';
  
      } catch (const std::exception& error) {
        std::cerr << "Invalid or incomplete data in " << file_path << '\n';
      return 1;
    }

    return 0;
  }

  
  auto readAmmosJSON(const std::string& file_path,  drone::DroneConfig& init_dr)-> int { 
	  std::ifstream ammos_json (file_path);
	  if (!ammos_json.is_open()) {
		  throw std::runtime_error("Unable to open " + file_path);
	  }
    ammo::Ammo* ammoTable = nullptr;

    try {
      json ammos;
      ammos_json >> ammos;

      size_t nAmmos = ammos.size();

      ammoTable = new ammo::Ammo[nAmmos];

      for (size_t i = 0; i < nAmmos; ++i)  {
            std::strncpy(ammoTable[i].name, ammos[i]["name"].get<std::string>().c_str(),31);
            ammoTable[i].mass = ammos[i]["mass"];
            ammoTable[i].drag = ammos[i]["drag"];
            ammoTable[i].lift = ammos[i]["lift"];     
      }
      init_dr.ammo_count = nAmmos;
      init_dr.ammo_table = ammoTable;
      return nAmmos;

    } catch (const std::exception& error) {
        std::cerr << "Invalid or incomplete data in " << file_path << '\n';
    }  

    if (ammoTable) {
      delete[] ammoTable;
    }
    return 0; 
  } 

  // Записати дані кроку у вихідн. JSON файл 
  auto pushStepToJSON(json& out, const drone::SimStep& sim_step)-> void { 
    json step; //крок х-дрона у-дрона кут-дрона стан-дрона ціль№
    step["position"] ={{"x", sim_step.pos.x}, {"y", sim_step.pos.y}};
    step["direction"] = sim_step.direction;
    step["state"] = sim_step.state;
    step["targetIndex"] = sim_step.targetIdx;
    if (sim_step.targetIdx >= 0) {
      step["dropPoint"] = {{"x", sim_step.dropPoint.x}, {"y", sim_step.dropPoint.y}};
      if (sim_step.state == drone::MOVING) {
        step["aimPoint"] = {{"x", sim_step.aimPoint.x}, {"y", sim_step.aimPoint.y}};
      } else {
        step["aimPoint"] = nullptr;
      }
      step["predictedTarget"] = {{"x", sim_step.predictedTarget.x}, {"y", sim_step.predictedTarget.y}};        
    } else { // no target, the fields not defined
      step["dropPoint"] = nullptr;
      step["aimPoint"] = nullptr;
      step["predictedTarget"] = nullptr;
    }
    out["steps"].push_back(step);
  }

  /* auto writeToJSONConf(const std::string& file_path, const sim::SimConfig& init_sim, 
                         const drone::DroneConfig& init_dr) { 

    json jsn;
    jsn["drone"]["position"]["x"] = init_dr.position.x;
    jsn["drone"]["position"]["y"] = init_dr.position.y;
    jsn["drone"]["altitude"] = init_dr.altitude;
    jsn["drone"]["initialDirection"] = init_dr.initial_direction;
    jsn["drone"]["attackSpeed"] = init_dr.attack_speed;
    jsn["drone"]["accelerationPath"] = init_dr.acceleration_path;
    jsn["drone"]["angularSpeed"] = init_dr.angular_speed;
    jsn["drone"]["turnThreshold"] = init_dr.turn_threshold;
     
    jsn["ammo"] = init_dr.ammo_name;
    jsn["simulation"]["timeStep"] = init_sim.time_step;
    jsn["simulation"]["hitRadius"] = init_dr.hit_radius;

    jsn["targetArrayTimeStep"] = init_sim.tgt_time_step;

    std::ofstream of_json(file_path);
    if (!of_json.is_open()) {
      throw std::runtime_error("Unable to open output file: " + file_path);
    }
     
    of_json << jsn.dump(2); // 2 spaces => tab
    return 0;
  }*/ //DONE

  /*auto writeTargetsJSON(const std::string& file_path, sim::Simulation& s) { 
    json jsn; 
    jsn["targetCount"] = kNtgts;
    jsn["timeSteps"] = sim::kTargetSteps;
    jsn["targets"] = json::array();
    
      for (auto i = 0; i < sim::kNtgts; ++i) { 
        json target = {
          {"positions",json::array()}
        };

        for (size_t j = 0; j < sim::kTargetSteps; ++j) { 
            target["positions"].push_back(
              {{"x", s.tgt_tracks[i].positions[j].x},
                    {"y", s.tgt_tracks[i].positions[j].y}
              });    
        }

        jsn["targets"].push_back(target);
      }
    
    std::ofstream of_json(file_path);
    if (!of_json.is_open()) {
      throw std::runtime_error("Unable to open output file: " + file_path);
    }
     
    of_json << jsn.dump(2); // 2 spaces => tab
    return 0;
  }*/ //DONE

} // namespace

//################################################################################
auto main() -> int {

  try {  
    json j_out;
    j_out["steps"] = json::array();
    sim::SimConfig sim_init{};
    drone::DroneConfig dr_init{};

    if (!readAmmosJSON(kAmmosPath, dr_init))
      return 1;

    if (0 != readJSONInput(kInputPath, sim_init, dr_init))
      return 1;    

    if (0 != readTargetsJSON(kTargetsPath, sim_init, dr_init))
      return 1;

    sim::Simulation sim{sim_init};
    drone::Drone dr{dr_init};  
       
    std::cout << std::fixed << std::setprecision(1);
 
    DEBUG("\tDr:" << dr.coord << " Dir" << dr.dirRad << ' ' << dr.droneStateToStr());
    DEBUG("\tAltitude,m: " << dr.alt
                  << "\n\tAttack_Speed,m/s: " << dr.attSpeed
                  << "\n\tAcceleration_path,m: " << dr.accPath
                  << "\n\tAngular_Speed,rad: " << dr.angSpeed
                  << "\n\tTurn_Threshold,rad: " << dr.turnThrld
                  << "\n\tAmmo: " << dr.ammo->name << ", m=" << dr.ammo->mass << ", d=" << dr.ammo->drag << ", l=" << dr.ammo->lift   
                  << "\n\tHit_Radius,m:#" << dr.hitRad
                  << " acc,m/ss: " << dr.kAcceleration);    

    int stepCurrent = 0;      //step, incremented through simulation until maximum
    double timeCurrent = 0.0; //current time 
    sim.initializeTgtPositions(dr);
    dr.startNewMission(sim.timeStep); //calculate first ballistic solutions, skip results

    for (size_t i = 0; i < dr.nTargets; ++i) DEBUG("#T " << dr.tgts[i]); //printAllTargets
  
    dr.updateSimStep(sim.simStep);
    pushStepToJSON(j_out, sim.simStep); //save step 0 data 

    do {        
      // move simulation time forward and check if we are not over max steps
      ++stepCurrent;  
      timeCurrent += sim.timeStep;  
      if(stepCurrent > kMaxSteps) { //simulation is over!
        LOG("No hit, simulation time is over!"); 
        break; 
      };

      //move targets and inform drone of their current positions, move drone according to its state, direction and speed
      sim.moveTargets(timeCurrent, dr);     
      dr.moveDrone(sim.timeStep);   

      // if we are on the way to target, continue mission 
      if (dr.isOnMission()) {
        DEBUG("\tT#" << dr.mission.tgtTag << ' ' << dr.tgts[dr.mission.tgtTag]);  
        DEBUG("\tOn" << dr.mission);

        dr.continueMission(sim.timeStep);   //analyze drone position on the drop path and change its state accordingly

        if (dr.mission.state == drone::FIRED) { //check  hit or miss
            double time_of_hit = dr.getAmmoFlyTime() + timeCurrent;
            //check if hit expected
            pointmath::Point tgt_pos_at_hit = sim.getTgtPositionAt(dr.mission.tgtTag, time_of_hit);
            double hit_dist = dr.getHitDistance(tgt_pos_at_hit);
            LOG("\tFired at: step#" << stepCurrent << "(" << timeCurrent << "s). Target coordinates at hit " << tgt_pos_at_hit 
                    << ", distance: " << hit_dist << 'm');

            if (dr.isTargetHit(hit_dist)) {
              LOG("Hit!");
            } else {
              LOG("Missed!"); 
            };          
        }
      }
      
      // if we are not on the mission (mission failed, or cant be achieved anymore), try to start new one 
      if (!dr.isOnMission()) { 
        dr.mission.tgtTag = dr.startNewMission(sim.timeStep);
        
        if (dr.isOnMission()) {         
          for (size_t i = 0; i < dr.nTargets; ++i) DEBUG("#T " << dr.tgts[i]); //printAllTargets
          LOG("\tStarting" << dr.mission);
        }
      }   

      DEBUG("#" << stepCurrent <<  " (" << timeCurrent  << "s) Dr:" << dr.coord 
                  << " Dir" << dr.dirRad << ' ' << dr.droneStateToStr() << " v=" 
                  << dr.speed << " err=" << dr.errcode);

      dr.updateSimStep(sim.simStep);
      pushStepToJSON(j_out, sim.simStep);  

    } while (dr.mission.state != drone::COMPLETED); 

    int result = dr.mission.state == drone::COMPLETED ? 0 : 1;

    j_out["totalSteps"] = stepCurrent + 1;

    DEBUG( " countMaxRecalc" << dr.countMaxRecalc);

    std::ofstream jf_out(kSimulationPath);
    if (jf_out.is_open()) { 
      jf_out << j_out.dump(2); // 2 spaces => tab
    } else {
      std::cerr << "Unable to open output file: " << kSimulationPath << '\n';
      result = 1;
    }

    dr.freeMemory();
    sim.freeMemory();
    return result;    
    
  } //eo try

  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }
  
}
