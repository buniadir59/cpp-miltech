#pragma once

#include "core_/TargetControl.hpp"
#include "core_/DroneControl.hpp"
#include "mission/MissionCtx.hpp"
#include "interfaces/IMissionState.hpp"
#include "dto/SimStatistics.hpp"
#include "dto/Ammo.hpp"
#include "drone/DronePhysics.hpp"

#include <memory>
#include <nlohmann/json.hpp>
#include <vector>
#include <atomic>

namespace dto {
struct MissionConfig;
}  // namespace dto

class ITargetProvider;
class IBallisticSolver;
class IConfigLoader;

/* TODO

MissionProcessor більше не зберігає і не інтегрує стан дрона — лише запитує телеметрію у фізики.

Стейт-машина (класи станів з ДЗ9) залишається в ньому

Метод run() — тіло потоку.
 */

namespace core {

// Приймає компоненти через вказівники на інтерфейси (патерн Стратегія)
// sets configuration, controls simulation steps,
// manages pool of Targets (updates coordinates and velocity from loader, maintaines adequate state )
class MissionProcessor {
  std::atomic<bool> threadReady{false};
  std::atomic<bool> threadStart{false};
  const double timeStep;

  ITargetProvider& targets_;
  std::unique_ptr<IBallisticSolver> solver_;
  drone::DronePhysics& drone_;  // TODO ptr  std::optional<DroneControl> drone;
  nlohmann::json j_out;

  mission::MissionCtx mctx;
  std::unique_ptr<IMissionState> mstate = nullptr;

  std::vector<TargetControl> targetDepo;

  // const dto::MissionConfig* mconf = nullptr;
  const dto::Ammo ammo;  // const dto::Ammo* ammo = nullptr;
  const int kMaxSteps;
  const std::string& simulationPath;
  dto::SimStatistics stats;  // includes steps, incremented through simulation until maximum

  auto updateTargets() -> void;  // get new targets position and velocity values

  auto pushStepToJSON(drone::DroneTelemetry& telemetry) -> void;  // Записати дані кроку у вихідн. JSON файл
  [[nodiscard]] auto isOnMission() const -> bool { return mctx.currentTgtTag >= 0; };

  auto init() -> void;  // Завантажити конфіг, підготувати дані ітерації
  [[nodiscard]] auto getInstantAimPoint(drone::DroneTelemetry& telemetry) -> pointmath::Point;

public:
  auto hasNext() -> bool;  // Перевірити, чи є ще необроблені цілі

  void changeSolver(std::unique_ptr<IBallisticSolver> solver)
  {
    solver_ = std::move(solver);
    // TODO   drone->setSolver(solver_.get());
  };  // Підмінити solver на льоту (Стратегія)

  bool step();  // Обробити наступну ціль: взяти дані з ITargetProvider, обчислити через IBallisticSolver,
                // return false if time is out

  auto getSimulationStatistics() -> const dto::SimStatistics&;

  auto isThreadReady() -> bool { return threadReady; };
  auto start() -> void { threadStart = true; };
  auto run() -> void;

  MissionProcessor(ITargetProvider& targets,
                   std::unique_ptr<IBallisticSolver> solver,
                   drone::DronePhysics& drone,
                   dto::Ammo ammo,
                   double time_step,
                   int kMaxSteps,
                   const std::string& simulationPath)
    : timeStep(time_step)
    , targets_(targets)
    , solver_(std::move(solver))
    , drone_(drone)
    , mctx(targetDepo)
    , ammo(ammo)
    , kMaxSteps(kMaxSteps)
    , simulationPath(simulationPath){};

  ~MissionProcessor();
};

}  // namespace core