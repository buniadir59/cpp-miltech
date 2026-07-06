#include "drone/DronePhysics.hpp"
#include "core_/TimeTracker.hpp"

#include "math/point_math.hpp"
#include "math/angle_math.hpp"

#include <memory>
#include <optional>
#include <thread>
#include <iostream>

namespace drone {

/* -sets threadReady;
   -waits for start;
   - step() in cycle every arrayTimeStep / timeS cale;
   -ends on request to stop.
   DronePhysics thread step()
        |
        | змінює unique_ptr<IDroneState> state
        | виконаує state і змінює DroneContext ctx
        v
Telemetry snapshot під mutex */
void DronePhysics::run() noexcept
{
  try {
  threadReady.store(true);
  while (!threadStart.load() && !threadStopRequested.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  
  TimeTracker& tt = TimeTracker::getInstance();

  while (!threadStopRequested.load()) {
    // read and apply command if any
    applyPendingCommands();

    // execute state
    if (auto nextState = state->execute(ctx); nextState != nullptr) {
      state = std::move(nextState);
    }
    // update telemetry snapshot
    {
      ctx.tel.timeSecSinceStart = tt.getElapsed();
      std::lock_guard<std::mutex> lock(telMutex);
      telSnapshot = ctx.tel;
    }

    auto wakeup = tt.nextWakeup(ctx.mconf.physicsTimeStep);
    std::this_thread::sleep_until(wakeup);
  } } catch (const std::exception& error) {
    std::cerr << "DronePhysics thread error: " << error.what() << '\n';
    failed.store(true);
    threadStopRequested.store(true);
  } catch (...) {
    std::cerr << "DronePhysics thread unknown error\n";
    failed.store(true);
    threadStopRequested.store(true);
  }
}

// apply latest command
void DronePhysics::applyPendingCommands()
{
  std::optional<dto::DroneCommand> latestCommand;

  while (auto command = commands_.try_pop()) {
    latestCommand = command;
  }

  if (latestCommand.has_value()) {
    transitionTo(latestCommand->state, latestCommand->angleSpeed);
  }
}

// apply pending commands one by one (might be needed if there is a  set of different types of commands)
void DronePhysics::applyEveryPendingCommand()
{
  while (auto command = commands_.try_pop()) {
    transitionTo(command->state, command->angleSpeed);
  }
}

// changes pointer to state that will be called next to required new state
// takes care if the required state can be achieved or transition state is necessary
// puts the value of required angular speed in the context
void DronePhysics::transitionTo(dto::DroneState new_state, float angle_speed)
{
  // cast to double and check limit before storing
  double angSpeed = static_cast<double>(angle_speed);
  if (std::abs(angSpeed) > ctx.mconf.maxAngularSpeedRadPerS) {
    angSpeed = angSpeed > 0 ? ctx.mconf.maxAngularSpeedRadPerS : -ctx.mconf.maxAngularSpeedRadPerS;
  }
  ctx.cmdAngSpeedRadPerS = angSpeed;
  if ((angSpeed == 0.0) || (new_state == dto::TURNING)) {  // fix the state to reflect reality
    new_state = dto::STOPPED;
  }
  switch (new_state) {
    case dto::STOPPED:                    // new state => stopped
      ctx.cmdAngSpeedRadPerS = 0.0;  // impose 0.0, state prevail
      if (ctx.plannedState == dto::TURNING) {
        state = std::make_unique<Stopped>();
      }
      else if ((ctx.plannedState == dto::ACCELERATING) || (ctx.plannedState == dto::MOVING)) {  // moving or accelerating
        state = std::make_unique<Decelerating>();
      }
      // if stopped or decelerating no need to change
      break;

    case dto::ACCELERATING:  // new state =>accelerating
      if ((ctx.plannedState != dto::ACCELERATING) && (ctx.plannedState != dto::MOVING)) {
        // any other(stopped/turning/decelerating)
        state = std::make_unique<Accelerating>();
      }
      break;

    case dto::MOVING:  // new state =>moving
      if ((ctx.plannedState != dto::MOVING) && (ctx.plannedState != dto::ACCELERATING)) {
        // any other(stopped/turning/decelerating)
        state = std::make_unique<Accelerating>();
      }
      break;

    case dto::DECELERATING:  // new state =>decelerating
      if ((ctx.plannedState == dto::ACCELERATING) || (ctx.plannedState == dto::MOVING)) {
        state = std::make_unique<Decelerating>();
      }
      // any other(stopped/turning/accelerating/decelerating)  no need to change
      break;

    case dto::TURNING:  // new state => turning
      if (ctx.plannedState == dto::STOPPED) {
        state = std::make_unique<Turning>();
      }
      else if ((ctx.plannedState == dto::MOVING) || (ctx.plannedState == dto::ACCELERATING)) {  // moving or accelerating
        state = std::make_unique<Decelerating>();
      }
      // turning or decelerating => no need to change
      break;
  }
}

// #####################################################################

// accelerate, it v= max, return moving
auto DroneContext::execAccelerating() -> std::unique_ptr<IDroneState>
{
  updateDir();                  //@exec accel
  tel.state = dto::ACCELERATING;     // currently executed
  plannedState = dto::ACCELERATING;  // next planned
  std::unique_ptr<IDroneState> nextState = nullptr;

  // increase speed. if attack speed, => state to Moving
  double v0 = static_cast<double>(tel.speed);
  double v1 = v0 + speedStep;
  if (v1 >= mconf.attackSpeed) {
    v1 = mconf.attackSpeed;
    plannedState = dto::MOVING;
    nextState = std::make_unique<Moving>();
  }
  tel.speed = v1;

  double dist = (v0 + v1) / 2.0 * mconf.physicsTimeStep;
  updateCoord(dist);

  return nextState;
}

// decelerate, it v= 0, return stopped or turning depending on ang speed value
auto DroneContext::execDecelerating() -> std::unique_ptr<IDroneState>
{
  updateDir();                  //@exec decel
  tel.state = dto::DECELERATING;     // currently executed
  plannedState = dto::DECELERATING;  // next planned
  std::unique_ptr<IDroneState> nextState = nullptr;

  // decrease speed. if 0, => state to stopped or turning
  double v0 = static_cast<double>(tel.speed);
  double v1 = v0 - speedStep;
  if (v1 < 0.0) {
    v1 = 0.0;
    if (cmdAngSpeedRadPerS == 0.0) {
      plannedState = dto::STOPPED;
      nextState = std::make_unique<Stopped>();
    }
    else {
      plannedState = dto::TURNING;
      nextState = std::make_unique<Turning>();
    }
  }
  tel.speed = v1;

  double dist = (v0 + v1) / 2.0 * mconf.physicsTimeStep;
  updateCoord(dist);

  return nextState;
}

auto DroneContext::updateCoord(double dist) -> void
{
  pointmath::Point delta = pointmath::cossin(static_cast<double>(tel.dir)) * dist;
  tel.x += static_cast<float>(delta.x);
  tel.y += static_cast<float>(delta.y);
}

// returns new direction based on current dir and angular speed with
// respect to its maximum value
auto DroneContext::updateDir() -> void
{
  if (cmdAngSpeedRadPerS != 0.0) {
    anglemath::AngleRad direction = static_cast<double>(tel.dir) + cmdAngSpeedRadPerS * mconf.physicsTimeStep;
    tel.dir = static_cast<float>(direction.value);
  }
}

}  // namespace drone