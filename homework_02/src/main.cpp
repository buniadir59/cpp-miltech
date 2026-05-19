#include "drone.hpp"
#include "point_math.hpp"
#include "simulation.hpp"

#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <string>
#include <cstring>

/* **** defines and contants **** */

//#define TEST_MODE

#define OUTPUT_FILE_NAME "simulation.txt"
#define TARGETS_FILE_NAME "targets.txt"
#define INPUT_FILE_NAME "input.txt"
#define DATA_FOLDER "homework_02/data/"

/* main.cpp - read input files, create objects, run simulation loop, save output */

namespace { 

  std::string kInputPath = DATA_FOLDER INPUT_FILE_NAME;
  std::string kTargetsPath = DATA_FOLDER TARGETS_FILE_NAME;
  std::string kOutputPath = DATA_FOLDER OUTPUT_FILE_NAME;

  // ## max number of simulation steps if any target not hit
#ifdef TEST_MODE
  constexpr int kMaxSteps = 1000; 
#else
  constexpr int kMaxSteps = 10000; 
#endif

  // **** print helpers
    
    double fixNegativeZero(double val, int precision) {
        double absV = std::abs(val);
        return absV < 1 / pow(10, precision) ? 0.0 : val;
    }

    auto rad2Grad(double ang) {  //utility for better human presentation
      return static_cast<int>(round(ang/M_PI*180)); 
    }

    std::ostream& operator<<(std::ostream& os, const pointmath::AngleRad& aR) { 
        return os << ",°: "<< rad2Grad(aR.value) << " ";
    }

    std::ostream& operator<<(std::ostream& os, const pointmath::Point& p) { 
        int prec = &os == &std::cout ? 1 : 2;
        return os << "( " << fixNegativeZero(p.x, prec) << ", " 
                         << fixNegativeZero(p.y, prec)  << " ) ";
    }

    std::ostream& operator<<(std::ostream& os, const ballistics::DropSolution& ds) { 
      pointmath::Point iP = {ds.intermediate_x, ds.intermediate_y };
      pointmath::Point fP ={ds.fire_x, ds.fire_y};
      if (ds.has_intermediate_point) {
        pointmath::Point iP = {ds.intermediate_x, ds.intermediate_y };
        return os << " I" << iP << " F" << fP;
      }
      return os << " F" << fP;
    } 

    std::ostream& operator<<(std::ostream& os, const drone::Mission& m) { 
      if (m.tgtTag < 0) return os << "=>No target";
      
      return os << "=>T#" << m.tgtTag << " dt,s: " << m.timeToDestination 
                << "  TA" << m.destAngle << " Dest" << m.destPoint 
                << ' ' << m.missionStateToStr(); 
    }
    
    std::ostream& operator<<(std::ostream& os, const drone::TargetState& tgt) { 
      return os << tgt.last_known << " V" << tgt.velocity 
                << "; Drop_route: "<< tgt.dropRoute << " total,s: " << tgt.time_total;
    }

    auto printDrone(const drone::Drone& dr, bool full) {     
      std::cout << "\tDr:" << dr.coord << " Dir" << dr.dirRad
                << ' ' << dr.droneStateToStr() << std::endl;
      if (full) {   //** constant values
        std::cout << "\tAltitude,m: " << dr.alt
                  << "\n\tAttack_Speed,m/s: " << dr.attSpeed
                  << "\n\tAcceleration_path,m: " << dr.accPath
                  << "\n\tAngular_Speed,rad: " << dr.angSpeed
                  << "\n\tTurn_Threshold,rad: " << dr.turnThrld
                  << "\n\tAmmo: " << dr.ammoName //.ammo.title << ", m=" << dr.ammo.mass << ", d=" << dr.ammo.drag << ", l=" << dr.ammo.lift   
                  << "\n\tHit_Radius,m:#" << dr.hitRad
                  << " t_acc,s: " << dr.kAccTime << " acc,m: " << dr.kAcc 
                  << std::endl;
      }
    } 

  auto printTargetState(int tgt_tag, const drone::TargetState& tgt) {           
    std::cout << "\tT#" << tgt_tag << ' ' << tgt << '\n';    
  }

    auto printAllTargets(const drone::Drone& dr) {
      for (auto i = 0; i < sim::kNtgts; ++i) {
        printTargetState(i, dr.tgts[i]);
      }
    }

  auto printNewMission(const drone::Drone& dr) {
    std::cout << "\tStarting" << dr.mission << '\n';
  }

  auto printMission(const drone::Drone& dr) {          
    std::cout << "\tOn" << dr.mission << '\n';
  }

  auto printMissionResults(bool result, 
          double time_of_hit, const pointmath::Point& tgt_pos_at_hit, double hit_dist) {
    const std::string  result_str = result ? "\tMissed!" : "\tHit!";
    std::cout << result_str << " at,s: " << time_of_hit << " on spot:" << tgt_pos_at_hit 
      << " Distance to target at hit,m: " << hit_dist << '\n';
  }

// **** helpers: read  input data from files write simulation output to file

  auto readTargetsFile(const std::string& file_path, sim::Simulation& s)  { 
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
  }

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
  }

  /* ***
  * Записати дані кроку у вихідн.файл
  * NB! format differs from requirents in HW doc. The change is agreed 
  */
  auto saveStep(std::ofstream& of, int stepCurrent, const drone::Drone& dr) { 
      //крок х-дрона у-дрона кут-дрона стан-дрона ціль№
      of  << stepCurrent << ' ' << dr.coord.x  << ' ' << dr.coord.y  << ' ' 
                  << dr.dirRad.value << ' ' << dr.state  << ' ' << dr.mission.tgtTag << '\n';
  }

  auto runSimulationAlarmClock(double& timeCurrent, int& stepCurrent, const sim::Simulation& sim)->bool {
 
      ++stepCurrent;  
      timeCurrent += sim.timeStep;  

      if (stepCurrent >= kMaxSteps) {
        std::cout << "No hit, simulation time is over!\n";
        return false;//not hit, simulation is over
      } 

      return true;
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
    readInputFile(kInputPath, sim_init, dr_init);

    sim::Simulation sim{sim_init};
    drone::Drone dr{dr_init};  
       
    readTargetsFile(kTargetsPath, sim);
    
    std::ofstream output_file(kOutputPath);
    if (!output_file.is_open()) {
      throw std::runtime_error("Unable to open output file: " + kOutputPath);
    }

    if (cout_to_file) {
      std::cout.rdbuf(output_file.rdbuf()); // Перенаправляємо cout у файл
    }

    std::cout << std::fixed << std::setprecision(1);
    output_file << std::fixed << std::setprecision(2);
 
    printDrone(dr, true);     

    bool isSimOn = true;
    int stepCurrent = 0;      //step, incremented through simulation until maximum
    double timeCurrent = 0.0; //current time 
    sim.initializeTgtPositions(dr);
    dr.startNewMission(sim.timeStep); //calculate first ballistic solutions, skip results

    //TODO for test ##################################
    printAllTargets(dr);

    // ###############################################
    
    saveStep(output_file, stepCurrent, dr); //save step 0 data

    while (isSimOn) {   
      
      // move simulation time forward and check if we are not over max steps
      isSimOn = runSimulationAlarmClock(timeCurrent, stepCurrent, sim); 

      //move targets and inform drone of their current positions, move drone according to its state, direction and speed
      sim.moveTargets(timeCurrent, dr);     
      dr.moveDrone(sim.timeStep);   

      // if we are on the way to target, continue mission 
      if (dr.isOnMission()) {
        printTargetState(dr.mission.tgtTag, dr.tgts[dr.mission.tgtTag]);

        printMission(dr);
        dr.continueMission(sim.timeStep);   //analyze drone position on the drop path and change its state accordingly
    //    printMission()); 

        if (dr.mission.state == drone::FIRED) { //check  hit or miss
            double time_of_hit = dr.getAmmoFlyTime() + timeCurrent;
            //check if hit expected
            pointmath::Point tgt_pos_at_hit = sim.getTgtPositionAt(dr.mission.tgtTag, time_of_hit);
            double hit_dist = dr.getHitDistance(tgt_pos_at_hit);
            isSimOn = !dr.isTargetHit(hit_dist);          
            printMissionResults(isSimOn, time_of_hit, tgt_pos_at_hit, hit_dist);    
        }
      }
      
      // if we are not on the mission (mission failed, or cant be achieved anymore), try to start new one 
      if (!dr.isOnMission()) { 
        dr.mission.tgtTag = dr.startNewMission(sim.timeStep);
        
        if (dr.isOnMission()) {
          printAllTargets(dr);
          printNewMission(dr);
        }
      }   

	    saveStep(output_file, stepCurrent, dr);

      // ############### test output     
      std::cout << "#" << stepCurrent <<  " (" << timeCurrent  << "s) "; 
      printDrone(dr, false);  
      // ###############   

    }; //eo while

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
