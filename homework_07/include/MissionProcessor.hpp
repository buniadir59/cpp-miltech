#pragma once

#include "dto/Ammo.hpp"
//#include "dto/DropSolution.hpp"
#include "TargetControl.hpp"
#include "dto/MissionConfig.hpp"
#include "dto/SimStatistics.hpp"
#include "DroneControl.hpp"
#include "Mission.hpp"

#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ISimulationClock.hpp"
#include "defines.hpp"

#include <optional>
#include <nlohmann/json.hpp>

namespace core {


// Приймає компоненти через вказівники на інтерфейси (патерн Стратегія)
// sets configuration, controls simulation steps, 
// manages pool of Targets (updates coordinates and velocity from loader, maintaines adequate state )
class MissionProcessor {

  ITargetProvider* targets_;      // IBallisticSolver* solver_;   
  IConfigLoader* loader_;       
  const ISimulationClock* clock{nullptr};

  nlohmann::json j_out;

  std::optional<DroneControl> drone; 

  Mission mission;

  int currentTgtTag = 0;
  dto::Ammo ammo;

  int stepCurrent = 0;       // step, incremented through simulation until maximum

  TargetControl targetDepo[defines::kMaxTargets]{}; 

  int target_count_{0};

  dto::SimStatistics stats{};  //counted stats: active, under attack,destroyed, total

  auto updateTargets() -> void;
 
  auto identifyNextTarget()-> bool; //return false if all destroyed
  auto getNextTarget() const -> int;
  
  auto fire() -> void;
  auto checkFireResult( TargetControl& tgt) -> bool;
  
  auto pushStepToJSON() -> void;  // Записати дані кроку у вихідн. JSON файл

 public:  

  void init(const char* configSource); //Завантажити конфіг через IConfigLoader, підготувати дані для ітерації
  auto hasNext() -> bool; //Перевірити, чи є ще необроблені цілі
  void reset()  { currentTgtTag = 0; };  //Почати ітерацію спочатку
  void changeSolver(IBallisticSolver* s) {mission.setSolver(s); }; //Підмінити solver на льоту (Стратегія)
  
  bool step();  // Обробити наступну ціль: взяти дані з ITargetProvider, обчислити через IBallisticSolver, 
                //return false if time is out 

  auto getSimulationStatistics() -> dto::SimStatistics&;

  const dto::MissionConfig* mconf = nullptr;

  MissionProcessor(ITargetProvider* targets,
                   IBallisticSolver* solver,
                   IConfigLoader* loader,
                  const ISimulationClock* clock) :
                  targets_(targets),
                  loader_(loader), 
                  clock(clock),
                  mission{solver}
  { };

  ~MissionProcessor();

};

} //eo namespace core