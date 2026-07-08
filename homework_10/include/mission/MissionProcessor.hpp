#pragma once

#include "mission/TargetControl.hpp"
#include "mission/MissionCtx.hpp"
#include "interfaces/IMissionState.hpp"
#include "dto/SimStatistics.hpp"
#include "dto/Ammo.hpp"
#include "drone/DronePhysics.hpp"

#include <memory>
#include <nlohmann/json.hpp>
#include <vector>
#include <atomic>
#include <string>

class ITargetProvider;
class IBallisticSolver;

/*
MissionProcessor більше не зберігає і не інтегрує стан дрона — лише запитує телеметрію у фізики.
Метод run() — тіло потоку.
 */

namespace core {

// Приймає компоненти через вказівники на інтерфейси (патерн Стратегія)
// sets configuration, controls simulation steps,
// manages pool of Targets (updates coordinates and velocity from loader, maintaines adequate state )
class MissionProcessor {
  std::atomic<bool> threadReady{false};
  std::atomic<bool> threadStart{false};
  std::atomic<bool> threadStopRequested{false};
  std::atomic<bool> failed{false};

  ITargetProvider& targets_;
  std::unique_ptr<IBallisticSolver> solver_;
  drone::DronePhysics& drone_;
  nlohmann::json j_out;

  std::vector<TargetControl> targetDepo;
  mission::MissionCtx mctx;
  std::unique_ptr<IMissionState> mstate = nullptr;


  const dto::Ammo ammo;
  const int kMaxSteps;
  const std::string simulationPath;
  dto::SimStatistics stats;  // includes steps, incremented through simulation until maximum

  auto updateTargets() -> void;   // get new targets position and velocity values
  auto pushStepToJSON() -> void;  // Записати дані кроку у вихідн. JSON файл
  [[nodiscard]] auto isOnMission() const -> bool { return mctx.currentTgtTag >= 0; };

  auto init() -> void;  // Завантажити конфіг, підготувати дані ітерації
  [[nodiscard]] auto getInstantAimPoint(dto::DroneTelemetry& telemetry) -> pointmath::Point;
  auto hasNext() -> bool { return stats.total != stats.destroyed; };  // Перевірити, чи є ще необроблені цілі
  bool step();  // Обробити наступну ціль: взяти дані з Target Provider, обчислити через IBallisticSolver,
                // return false if time is out
  auto updateBasicAmmoRes() -> void;

public:
  void changeSolver(std::unique_ptr<IBallisticSolver> solver) { solver_ = std::move(solver); };  // Підмінити solver на льоту (Стратегія)

  [[nodiscard]] auto getSimulationStatistics() -> const dto::SimStatistics&;
  [[nodiscard]] auto isThreadReady() -> bool { return threadReady.load(); };
  [[nodiscard]] auto hasFailed() const -> bool { return failed.load(); };
  auto start() -> void { threadStart.store(true); };
  auto stop() -> void { threadStopRequested.store(true); };
  auto run() noexcept -> void;

  MissionProcessor(const dto::MissionConfig& mconf,
                   ITargetProvider& targets,
                   std::unique_ptr<IBallisticSolver> solver,
                   drone::DronePhysics& drone,
                   dto::Ammo ammo,
                   int kMaxSteps,
                   const std::string simulationPath)
    : targets_(targets)
    , solver_(std::move(solver))
    , drone_(drone)
    , mctx(mconf, targetDepo)
    , ammo(ammo)
    , kMaxSteps(kMaxSteps)
    , simulationPath(simulationPath){};

  ~MissionProcessor();
};

}  // namespace core