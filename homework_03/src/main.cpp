#include "drone.hpp"
#include "point_math.hpp"
#include "simulation.hpp"

#include "json.hpp"
#include <cstddef>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <string>
#include <cstring>

using json = nlohmann::json;
using Point = pointmath::Point;

/* **** defines and contants **** */

//#define TEST_MODE
#define ENABLE_LOG 1 
#define ENABLE_DEBUG 1

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
  std::string kAmmosPath = "homework_03/data/ammo.json";
  std::string kInputPath = "homework_03/data/config.json";
  std::string kTargetsPath = "homework_03/data/targets.json";
  std::string kOutputPath = "homework_03/data/simulation.txt"; //TODO json";

  // ## max number of simulation steps if any target not hit
#ifdef TEST_MODE
  constexpr int kMaxSteps = 1000; 
#else
  constexpr int kMaxSteps = 10000; 
#endif


// **** helpers: read  input data from files write simulation output to file

  /*auto readTargetsFile(const std::string& file_path, sim::Simulation& s)  { 
	  std::ifstream i_file (file_path);
	  if (!i_file.is_open()) {
		  throw std::runtime_error("Unable to open " + file_path);
	  }

    for (auto i = 0; i < sim::kNtgts; ++i) for (size_t j = 0; j < sim::kTargetSteps; ++j) {
        i_file >> s.tgt_tracks[i].positions[j].x;    
    }
    
    for (auto i = 0; i < sim::kNtgts; ++i) for (size_t j = 0; j < sim::kTargetSteps; ++j) {
        i_file >> s.tgt_tracks[i].positions[j].y; 
    }

    if (!i_file) {
      throw std::runtime_error("Invalid or incomplete data in "  + file_path );
    }

    return 0;
  }*/

  /*
  auto readInputFile(const std::string& file_path, sim::SimConfig& init_sim,drone::DroneConfig& init_dr) { 
    std::ifstream input_file(file_path);

    if (!input_file.is_open()) {
      throw std::runtime_error("Unable to open input file: " + file_path);
    }

    input_file >> init_dr.position.x >> init_dr.position.y >> init_dr.altitude
              >> init_dr.initial_direction >> init_dr.attack_speed >> init_dr.acceleration_path
              >> init_dr.ammo_name >> init_sim.tgt_time_step >> init_sim.time_step 
              >> init_dr.hit_radius >> init_dr.angular_speed >> init_dr.turn_threshold;

    if (!input_file) {
      throw std::runtime_error("Input file has invalid or incomplete data");
    }

    return 0;
  } */

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
  }*/

  auto readTargetsJSON(const std::string& file_path, sim::SimConfig& init_sim) { 
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
      init_sim.target_steps = timeSteps;   
      init_sim.target_tracks = coords;

    } catch (const std::exception& error) {
        std::cerr << "Invalid or incomplete data in " << file_path << '\n';
        //??? TODO for (size_t i = 0; i < tgtsCount; ++i) { delete[] tgtTracks[i]; }; delete[] tgtTracks; ;
      return 1;
    }  

    return 0;
  } 

  auto readJSONInput(const std::string& file_path, sim::SimConfig& init_sim,drone::DroneConfig& init_dr) { 
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
      init_dr.ammo_name = jsn["ammo"];
  
      } catch (const std::exception& error) {
        std::cerr << "Invalid or incomplete data in " << file_path << '\n';
      return 1;
    }

    return 0;
  }

//return number of ammos
auto readAmmosJSON(const std::string& file_path, sim::SimConfig& init_sim)-> int { 
	  std::ifstream ammos_json (file_path);
	  if (!ammos_json.is_open()) {
		  throw std::runtime_error("Unable to open " + file_path);
	  }
    sim::Ammo* ammoTable = nullptr;

    try {
      json ammos;
      ammos_json >> ammos;

      size_t nAmmos = ammos.size();

      ammoTable = new sim::Ammo[nAmmos];

      for (size_t i = 0; i < nAmmos; ++i)  {
            std::strncpy(ammoTable[i].name, ammos[i]["name"].get<std::string>().c_str(),31);
            ammoTable[i].mass = ammos[i]["mass"];
            ammoTable[i].drag = ammos[i]["drag"];
            ammoTable[i].lift = ammos[i]["lift"];     
      }
      init_sim.number_of_ammos = nAmmos;
      init_sim.ammo_table = ammoTable;
      return nAmmos;

    } catch (const std::exception& error) {
        std::cerr << "Invalid or incomplete data in " << file_path << '\n';
        //??? TODO for (size_t i = 0; i < tgtsCount; ++i) { delete[] tgtTracks[i]; }; delete[] tgtTracks; ;
    }  

    if (ammoTable) delete[] ammoTable;
    return 0; // ammo; //TODO
  } 

  /* ***
  * Записати дані кроку у вихідн. JSON файл

    auto saveStepToJSON(json& out, const drone::Drone& dr) { 
    json step; //крок х-дрона у-дрона кут-дрона стан-дрона ціль№
    step["position"] ={{"x", dr.coord.x}, {"y", dr.coord.y}};
    step["direction"] = dr.dirRad.value;
    step["state"] = dr.state;
    step["targetIndex"] = dr.mission.tgtTag;
     
    out["steps"].push_back(step);
  }
  */
  auto saveStep(std::ofstream& of, int stepCurrent, const drone::Drone& dr) { 
      //крок х-дрона у-дрона кут-дрона стан-дрона ціль№
      of  << stepCurrent << ' ' << dr.coord.x  << ' ' << dr.coord.y  << ' ' 
                  << dr.dirRad.value << ' ' << dr.state  << ' ' << dr.mission.tgtTag << '\n';
  }


} // namespace

//################################################################################
auto main() -> int {

  bool cout_to_file = false; // Умова: true для файлу, false для консолі
  std::streambuf* original_buf = nullptr;

#ifdef TEST_MODE //save console to file
  cout_to_file = true;
  original_buf = std::cout.rdbuf(); // Зберігаємо оригінальний буфер консолі
#endif

  try {  

    sim::SimConfig sim_init{};
    drone::DroneConfig dr_init{};

    if (!readAmmosJSON(kAmmosPath, sim_init))
      return 1;

    if (0 != readJSONInput(kInputPath, sim_init, dr_init))
      return 1;    

    if (0 != readTargetsJSON(kTargetsPath, sim_init))
      return 1;

    sim::Simulation sim{sim_init};
    drone::Drone dr{dr_init};  
       
    // readTargetsJSON("homework_03/data/targets.json", sim); // readTargetsFile(kTargetsPath, sim); // writeTargetsJSON("homework_03/data/targets.json", sim);

    std::ofstream output_file(kOutputPath);
    if (!output_file.is_open()) {
      throw std::runtime_error("Unable to open output file: " + kOutputPath);
    }

    if (cout_to_file) {
      std::cout.rdbuf(output_file.rdbuf()); // Перенаправляємо cout у файл
    }

    std::cout << std::fixed << std::setprecision(1);
    output_file << std::fixed << std::setprecision(2);
 
    DEBUG("\tDr:" << dr.coord << " Dir" << dr.dirRad << ' ' << dr.droneStateToStr());
    DEBUG("\tAltitude,m: " << dr.alt
                  << "\n\tAttack_Speed,m/s: " << dr.attSpeed
                  << "\n\tAcceleration_path,m: " << dr.accPath
                  << "\n\tAngular_Speed,rad: " << dr.angSpeed
                  << "\n\tTurn_Threshold,rad: " << dr.turnThrld
                  << "\n\tAmmo: " << dr.ammoName //.ammo.title << ", m=" << dr.ammo.mass << ", d=" << dr.ammo.drag << ", l=" << dr.ammo.lift   
                  << "\n\tHit_Radius,m:#" << dr.hitRad
                  << " t_acc,s: " << dr.kAccTime << " acc,m: " << dr.kAcc);    

    int stepCurrent = 0;      //step, incremented through simulation until maximum
    double timeCurrent = 0.0; //current time 
    sim.initializeTgtPositions(dr);
    dr.startNewMission(sim.timeStep); //calculate first ballistic solutions, skip results

#ifdef TEST_MODE 
    printAllTargets(dr);
#endif    
 //printAllTargets
  for (drone::TargetState tgt : dr.tgts) DEBUG("#T " << tgt);

    
    saveStep(output_file, stepCurrent, dr); //save step 0 data //TODO

    do {        
      // move simulation time forward and check if we are not over max steps
      ++stepCurrent;  
      timeCurrent += sim.timeStep;  

      DEBUG("#" << stepCurrent <<  " (" << timeCurrent  << "s) "); 

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
            LOG("\tFired at: " << timeCurrent << "s. Target coordinates at hit " << tgt_pos_at_hit 
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
          //printAllTargets
          for (drone::TargetState tgt : dr.tgts) DEBUG("#T " << tgt);
          LOG("\tStarting" << dr.mission);
        }
      }   

	    saveStep(output_file, stepCurrent, dr); //TODO
  
      DEBUG("\tDr:" << dr.coord << " Dir" << dr.dirRad << ' ' << dr.droneStateToStr());    

    } while (dr.mission.state != drone::COMPLETED); 

    if (cout_to_file) {
      std::cout.rdbuf(original_buf); // Обов'язково повертаємо оригінальний буфер 
    }

    output_file.close();
    return dr.mission.state == drone::COMPLETED ? 0 : 1;    
    
  } //eo try

  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }
  
}
