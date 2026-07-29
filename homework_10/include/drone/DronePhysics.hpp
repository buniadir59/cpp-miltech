#pragma once

#include "dto/DroneInterfaceStructures.hpp"
#include "dto/MissionConfig.hpp"
#include "interfaces/IDroneState.hpp"
#include "drone/ThreadSafeQueue.hpp"

#include <cmath>
#include <atomic>
#include <memory>

/*
клас фізики дрона . Володіє станом дрона, приймає команди через чергу і у власному потоці кожні physicsTimeStep інтегрує рух.
state змінюється тільки в DronePhysics thread;
MissionProcessor надсилає DroneCommand у ThreadSafeQueue;
MissionProcessor читає тільки DroneTelemetry snapshot;
getTelemetry() захищений mutex;
command queue захищена своїм mutex;
state і ctx не доступні напряму ззовні.
 */

namespace drone {
/* struct DroneConfig {  // contains constant configuration params based on preloaded configuration
  double accelerationPath;
  double altitude;
  double attackSpeed;
  double maxAngularSpeedRadPerS;  // this is max angular speed => threshold
  //  double turnThreshold;    // no sense in this parameter for HW10 as dt is much less now
  double physicsTimeStep;  // 0.01
  double acceleration;
  double speedStep;  // increase/decrease speed at one physics time step
  double distStep;   // distance at one phys step under attack speed
}; */

struct DroneContext {
  const dto::MissionConfig& mconf;

  double acceleration;
  double speedStep;                 // increase/decrease speed at one physics time step
  double distStep;                  // distance at one phys step under attack speed
  dto::DroneTelemetry tel;          // actual telemetry block
  double cmdAngSpeedRadPerS = 0.0;  // angular speed from latest applied command
  dto::DroneState plannedState;

  DroneContext(const dto::MissionConfig& mconf)
    : mconf(mconf)
  {
    acceleration = mconf.attackSpeed * mconf.attackSpeed / (mconf.kAccelerationPath * 2.0);
    speedStep = acceleration * mconf.physicsTimeStep;
    distStep = mconf.attackSpeed * mconf.physicsTimeStep;
  }

  auto updateDir() -> void;
  auto updateCoord(double dist) -> void;

  auto execAccelerating() -> std::unique_ptr<IDroneState>;  // next state can be moving
  auto execDecelerating() -> std::unique_ptr<IDroneState>;  // next state can be turning or stopped
};

class Stopped final : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(DroneContext& ctx) override
  {  // final state,  can be changed by external cmd only
    ctx.plannedState = dto::STOPPED;
    ctx.tel.state = dto::STOPPED;
    return nullptr;
  };
};

class Turning final : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(drone::DroneContext& ctx) override
  {                                   // final state, can be changed by external cmd only
    ctx.updateDir();                  // turn
    ctx.tel.state = dto::TURNING;     // report what has been executed
    ctx.plannedState = dto::TURNING;  // next planned
    return nullptr;
  };
};

class Accelerating final : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(drone::DroneContext& ctx) override { return ctx.execAccelerating(); };
};

class Decelerating final : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(drone::DroneContext& ctx) override { return ctx.execDecelerating(); };
};

class Moving final : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(drone::DroneContext& ctx) override
  {                   // final state, can be changed by external cmd only
    ctx.updateDir();  // turn on the flight and if needed
    ctx.updateCoord(ctx.distStep);

    ctx.tel.state = dto::MOVING;     // what has been executed
    ctx.plannedState = dto::MOVING;  // next planned
    return nullptr;                  // only by cmd we can change the state
  };
};

// ############################################################################
class DronePhysics {
  std::unique_ptr<IDroneState> state;

 // const dto::DroneConfig drConf;  // Dron configuration constants from app config and based on them
  drone::DroneContext ctx;

  std::atomic<bool> threadStopRequested{false};
  std::atomic<bool> threadStart{false};
  std::atomic<bool> threadReady{false};
  std::atomic<bool> failed{false};

  ThreadSafeQueue<dto::DroneCommand> commands_;
  mutable std::mutex telMutex;
  dto::DroneTelemetry telSnapshot;

  void applyPendingCommands();
  void applyEveryPendingCommand();  // alternative,might be needed
  void transitionTo(dto::DroneState new_state, float angle_speed);

public:
  DronePhysics(const dto::MissionConfig& config)
    : /* drConf({config.accelerationPath,
              config.altitude,
              config.attackSpeed,
              config.angularSpeed,
              config.physicsTimeStep,
              config.attackSpeed * config.attackSpeed / (config.accelerationPath * 2.0),
              config.attackSpeed * config.attackSpeed / (config.accelerationPath * 2.0) * config.physicsTimeStep,
              config.attackSpeed * config.physicsTimeStep})
    ,  */ctx(config)
  {
    ctx.tel = {0.0,
               float(config.initialPosition.x),
               float(config.initialPosition.y),
               static_cast<float>(config.kAltitude),
               0.0,
               static_cast<float>(config.initialDirection),
               dto::STOPPED};
    telSnapshot = ctx.tel;
    state = std::make_unique<Stopped>();  // initial state is full stop
    ctx.plannedState = dto::STOPPED;
  }

  auto start() -> void { threadStart.store(true); };
  auto stop() -> void { threadStopRequested.store(true); };
  [[nodiscard]] auto isThreadReady() const -> bool { return threadReady.load(); };
  [[nodiscard]] auto hasFailed() const -> bool { return failed.load(); }
  auto run() noexcept -> void;

  auto sendCommand(dto::DroneCommand& command) -> void { commands_.push(command); };
  [[nodiscard]] auto getTelemetry() const -> dto::DroneTelemetry
  {
    std::lock_guard<std::mutex> lock(telMutex);
    return telSnapshot;
  };
};

}  // namespace drone