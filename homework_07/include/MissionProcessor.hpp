#pragma once

#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/ISimulationClock.hpp"
#include "config/defines.hpp"
#include "dto/Ammo.hpp"
#include "dto/MissionConfig.hpp"
#include "dto/SimStatistics.hpp"
#include "TargetControl.hpp"
#include "DroneControl.hpp"
#include "Mission.hpp"

#include <optional>
#include <nlohmann/json.hpp>

namespace core {

// Приймає компоненти через вказівники на інтерфейси (патерн Стратегія)
// sets configuration, controls simulation steps,
// manages pool of Targets (updates coordinates and velocity from loader, maintaines adequate state )
class MissionProcessor {
  ITargetProvider* targets_;
  IConfigLoader* loader_;
  const ISimulationClock* simClock{nullptr};

  nlohmann::json j_out;

  std::optional<DroneControl> drone;

  Mission mission;

  int currentTgtTag = 0;
  dto::Ammo ammo;

  int stepCurrent = 0;  // step, incremented through simulation until maximum

  TargetControl targetDepo[defines::kMaxTargets]{};

  int target_count_{0};

  dto::SimStatistics stats{};  // counted stats: active, under attack,destroyed, total
  const dto::MissionConfig* mconf = nullptr;

  auto updateTargets() -> void;

  auto identifyNextTarget() -> bool;  // return false if all destroyed
  auto getNextTarget() const -> int;

  auto fire() -> void;
  auto checkFireResult(TargetControl& tgt) -> bool;

  auto pushStepToJSON() -> void;  // Записати дані кроку у вихідн. JSON файл

public:
  auto init(const char* configSource) -> const dto::MissionConfig*;  // Завантажити конфіг через IConfigLoader, підготувати дані для
                                                                     // ітерації
  auto hasNext() -> bool;               // Перевірити, чи є ще необроблені цілі
  void reset() { currentTgtTag = 0; };  // Почати ітерацію спочатку
  void changeSolver(IBallisticSolver* solver) { mission.setSolver(solver); };  // Підмінити solver на льоту (Стратегія)

  bool step();  // Обробити наступну ціль: взяти дані з ITargetProvider, обчислити через IBallisticSolver,
                // return false if time is out

  auto getSimulationStatistics() -> dto::SimStatistics&;

  MissionProcessor(ITargetProvider* targets, IBallisticSolver* solver, IConfigLoader* loader, const ISimulationClock* clock)
    : targets_(targets)
    , loader_(loader)
    , simClock(clock)
    , mission{solver} {};

  ~MissionProcessor();

  
};

}  // namespace core