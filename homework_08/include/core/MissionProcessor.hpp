#pragma once

#include "core/TargetControl.hpp"
#include "core/DroneControl.hpp"
#include "core/Mission.hpp"

#include <optional>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>

namespace dto {
struct MissionConfig;
struct Ammo;
}  // namespace dto

class ITargetProvider;
class IBallisticSolver;
class IConfigLoader;
class ISimulationClock;

namespace dto {
struct SimStatistics;
}

namespace core {

// Приймає компоненти через вказівники на інтерфейси (патерн Стратегія)
// sets configuration, controls simulation steps,
// manages pool of Targets (updates coordinates and velocity from loader, maintaines adequate state )
class MissionProcessor {
  ITargetProvider* targets_;
  IConfigLoader* loader_;
  ISimulationClock* simClock{nullptr};

  nlohmann::json j_out;

  std::optional<DroneControl> drone;

  Mission mission;

  int currentTgtTag = 0;

  int stepCurrent = 0;  // step, incremented through simulation until maximum

  std::vector<TargetControl> targetDepo;

  const dto::MissionConfig* mconf = nullptr;
  const dto::Ammo* ammo = nullptr;

  auto updateTargets() -> void;

  auto identifyNextTarget() -> bool;  // return false if all destroyed
  auto getNextTarget() const -> int;

  auto fire() -> void;
  auto checkFireResult(TargetControl& tgt) -> bool;

  auto pushStepToJSON() -> void;  // Записати дані кроку у вихідн. JSON файл

public:
  auto init(const std::string& configSource) -> const dto::MissionConfig*;  // Завантажити конфіг через IConfigLoader, підготувати дані для
                                                                            // ітерації
  auto hasNext() -> bool;               // Перевірити, чи є ще необроблені цілі
  void reset() { currentTgtTag = 0; };  // Почати ітерацію спочатку
  void changeSolver(IBallisticSolver* solver) { mission.setSolver(solver); };  // Підмінити solver на льоту (Стратегія)

  bool step();  // Обробити наступну ціль: взяти дані з ITargetProvider, обчислити через IBallisticSolver,
                // return false if time is out

  auto getSimulationStatistics() -> dto::SimStatistics;

  MissionProcessor(ITargetProvider* targets, IBallisticSolver* solver, IConfigLoader* loader, ISimulationClock* clock)
    : targets_(targets)
    , loader_(loader)
    , simClock(clock)
    , mission{solver} {};

  ~MissionProcessor();
};

}  // namespace core