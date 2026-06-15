#pragma once

#include "dto/SimStatistics.hpp"
#include "core/TargetControl.hpp"
#include "core/DroneControl.hpp"
#include "core/Mission.hpp"

#include <optional>
#include <memory>
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

namespace core {

// Приймає компоненти через вказівники на інтерфейси (патерн Стратегія)
// sets configuration, controls simulation steps,
// manages pool of Targets (updates coordinates and velocity from loader, maintaines adequate state )
class MissionProcessor {
  std::unique_ptr<ITargetProvider> targets_;
  std::unique_ptr<IConfigLoader> loader_;
  std::unique_ptr<IBallisticSolver> solver_;
  std::unique_ptr<ISimulationClock> clock_;

  nlohmann::json j_out;

  std::optional<DroneControl> drone;

  Mission mission;

  int currentTgtTag = 0;

  int stepCurrent = 0;  // step, incremented through simulation until maximum

  std::vector<TargetControl> targetDepo;

  dto::SimStatistics stats{};
  const dto::MissionConfig* mconf = nullptr;
  const dto::Ammo* ammo = nullptr;

  auto updateTargets() -> void;

  auto identifyNextTarget() -> bool;  // return false if all destroyed
  auto getNextTarget() const -> int;

  auto fire() -> void;
  auto checkFireResult(TargetControl& tgt) -> bool;

  auto pushStepToJSON() -> void;  // Записати дані кроку у вихідн. JSON файл

public:
  auto init(const std::string& configSource) -> void; // Завантажити конфіг, підготувати дані ітерації
  auto hasNext() -> bool;               // Перевірити, чи є ще необроблені цілі
  void reset() { currentTgtTag = 0; };  // Почати ітерацію спочатку
  void changeSolver(std::unique_ptr<IBallisticSolver> solver)
  {
    solver_ = std::move(solver);
    mission.setSolver(solver_.get());
  };  // Підмінити solver на льоту (Стратегія)

  bool step();  // Обробити наступну ціль: взяти дані з ITargetProvider, обчислити через IBallisticSolver,
                // return false if time is out

  auto getSimulationStatistics() -> dto::SimStatistics&;

  MissionProcessor(std::unique_ptr<ITargetProvider> targets, 
    std::unique_ptr<IBallisticSolver> solver, 
    std::unique_ptr<IConfigLoader> loader,  std::unique_ptr<ISimulationClock> clock)
    : targets_(std::move(targets))
    , loader_(std::move(loader))
    , solver_(std::move(solver))
    , clock_(std::move(clock))
    , mission{solver_.get()} {};

  ~MissionProcessor();
};

}  // namespace core