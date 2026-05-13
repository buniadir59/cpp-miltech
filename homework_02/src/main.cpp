#include "drone.hpp"
#include "simulation.hpp"

#include <exception>
#include <fstream>
//#include <iomanip>
#include <iostream>
#include <cmath>
#include <string>
#include <cstring>

/* **** defines and contants **** */

#define TEST_NUM_STEPS  

#define OUTPUT_FILE_NAME "simulation.txt"
#define TARGETS_FILE_NAME "targets.txt"
#define INPUT_FILE_NAME "input.txt"
#define DATA_FOLDER "homework_02/data/"

/*
main.cpp
  read input
  read targets
  run simulation
  manage output 
*/
namespace { 

  std::string kInputPath = DATA_FOLDER INPUT_FILE_NAME;
  std::string kTargetsPath = DATA_FOLDER TARGETS_FILE_NAME;
  std::string kOutputPath = DATA_FOLDER OUTPUT_FILE_NAME;

  // ## max number of simulation steps if any target not hit
#ifdef TEST_NUM_STEPS
  constexpr int kMaxSteps = 100; 
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
        return os << rad2Grad(aR.value);
    }

    std::ostream& operator<<(std::ostream& os, const pointmath::Point& p) { 
        int prec = &os == &std::cout ? 1 : 2;
        return os << "(" << fixNegativeZero(p.x, prec) << ", " 
                         << fixNegativeZero(p.y, prec)  << ")";
    }

    
    auto printDrone(const drone::Drone& dr, bool full) {
      //** changing during simulation
      std::cout << "Dr: " << dr.coord << " Dir=" << dr.dirRad
                << ' ' + getDroneStateStr(dr.state) << std::endl;
      if (full) {   //** constant values
          std::cout << "\tAltitude, m: " << dr.alt
                  << "\n\tAttack Speed, m/s: " << dr.attSpeed
                  << "\n\tAcceleration path, m: " << dr.accPath
                  << "\n\tAngular Speed, rad: " << dr.angSpeed
                  << "\n\tTurn Threshold, rad: " << dr.turnThrld
                  << "\n\tAmmo: " << dr.ammoName //.ammo.title << ", m=" << dr.ammo.mass << ", d=" << dr.ammo.drag << ", l=" << dr.ammo.lift   
                  << "\n\tHit Radius, m: " << dr.hitRad
                //  << "\n\tt_ff=" << dr.ammoFFtime << " hd_ff=" << dr.ammoFFdist << " minDist=" << (dr.ammoFFdist + dr.accPath)
                //  << "\n\tturn180=" << dr.kTimeTurn180 << " t_acc=" << dr.kAccTime << " acc=" << dr.kAcc 
                  << std::endl;
      }
    } 

  /*
    auto printDropSolution(const ballistics::DropSolution& solution) {
      std::cout << std::fixed << std::setprecision(3);

      std::cout << "ft_s " << solution.fall_time_s << " fhd_m=" << solution.horizontal_fall_distance_m << ' ';

      if (solution.has_intermediate_point) {
        std::cout << solution.intermediate_x << ' ' << solution.intermediate_y ;
      }

      std::cout << " " << solution.fire_x << ' ' << solution.fire_y << '\n';
    }  */

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

    double initial_dir{};
    input_file >> init_dr.position.x >> init_dr.position.y >> init_dr.altitude
              >> initial_dir >> init_dr.attack_speed >> init_dr.acceleration_path
              >> init_dr.ammo_name >> init_sim.tgt_time_step >> init_sim.time_step 
              >> init_dr.hit_radius >> init_dr.angular_speed >> init_dr.turn_threshold;
              
    init_dr.initial_direction = pointmath::AngleRad{initial_dir};

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
      of  << stepCurrent << ' ' << dr.coord.x << ' ' << dr.coord.y << ' ' 
                  << dr.dirRad << ' ' << dr.state << ' ' << dr.currentTgtTag << '\n';
  }

} // namespace

//################################################################################
auto main() -> int {

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

    bool hit = false;
    int stepCurrent = 0;    //step, incremented through simulation until maximum
    double timeCurrent = 0; //current time 
   
    //TODO for test 
    sim.timeStep = 1.0;
    printDrone(dr, true);

    while (!hit) {   
      if (stepCurrent >= kMaxSteps) {
        std::cout << "No hit, simulation time is over!\n";
        output_file.close();
        return 1;
      } 
   
      sim.updateTargetsPosition(timeCurrent, dr);

      //update drone position   //TODO  updateDronePosition();    
      //reevaluate best target   //TODO reevaluateBestTarget();  
      //analyze drone position re current mission and change its state accordingly //TODO steerDrone(); 

	    saveStep(output_file, stepCurrent, dr);

      //test output  //   printDrone(false);    
      std::cout << "T0:" << dr.tgts[0].last_known << ' ' << dr.tgts[0].predictPositionAt(timeCurrent); 
      std::cout <<  " Time=" << timeCurrent << ", steps=" << stepCurrent << std::endl;

      // move simulation 
      ++stepCurrent;  
      timeCurrent += sim.timeStep;  
    }; //eo while

  }
  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
