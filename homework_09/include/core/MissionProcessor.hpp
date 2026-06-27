#pragma once

#include "core/TargetControl.hpp"
#include "core/DroneControl.hpp"
#include "mission/MissionCtx.hpp"
#include "interfaces/IMissionState.hpp"
#include "dto/SimStatistics.hpp"

#include <optional>
#include <memory>
#include <nlohmann/json.hpp>
#include <vector>

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
  const ISimulationClock* simClock{nullptr};  // const - only reads time

  nlohmann::json j_out;

  std::optional<DroneControl> drone;
  mission::MissionCtx mctx;
  std::unique_ptr<IMissionState> mstate = nullptr;

  std::vector<TargetControl> targetDepo;

  const dto::MissionConfig* mconf = nullptr;
  const dto::Ammo* ammo = nullptr;
  dto::SimStatistics stats{};  // includes steps, incremented through simulation until maximum

  auto updateTargets() -> void;  // get new targets position and velocity values

  auto pushStepToJSON() -> void;  // Записати дані кроку у вихідн. JSON файл
  [[nodiscard]] auto isOnMission() const -> bool { return mctx.currentTgtTag >= 0; };

public:
  auto init() -> const dto::MissionConfig*;  // Завантажити конфіг, підготувати дані ітерації
  auto hasNext() -> bool;                    // Перевірити, чи є ще необроблені цілі

  void changeSolver(std::unique_ptr<IBallisticSolver> solver)
  {
    solver_ = std::move(solver);
    drone->setSolver(solver_.get());
  };  // Підмінити solver на льоту (Стратегія)

  bool step();  // Обробити наступну ціль: взяти дані з ITargetProvider, обчислити через IBallisticSolver,
                // return false if time is out

  auto getSimulationStatistics() -> const dto::SimStatistics&;

  MissionProcessor(std::unique_ptr<ITargetProvider> targets,
                   std::unique_ptr<IBallisticSolver> solver,
                   std::unique_ptr<IConfigLoader> loader,
                   ISimulationClock* clock)
    : targets_(std::move(targets))
    , loader_(std::move(loader))
    , solver_(std::move(solver))
    , simClock(clock)
    , mctx(targetDepo, clock){};

  ~MissionProcessor();
};

}  // namespace core