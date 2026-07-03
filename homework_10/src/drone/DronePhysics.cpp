#include "drone/DronePhysics.hpp"
#include "core_/TimeTracker.hpp"

namespace drone {

/* -sets threadReady;
   -waits for start;
   -calls update() every arrayTimeStep / timeS cale;
   -ends on request to stop. */
void DronePhysics::run()
{
  threadReady = true;
  while (!threadStart) {
  };

  TimeTracker& tt = TimeTracker::getInstance();
  double time = 0.0;
  int step = 0;
  while (!threadStopRequested) {
    time = tt.getElapsed();
    int next = std::floor(time / physicsTimeStep);
    if (next != step) {
      step = next;
      update();
    }
  }
 // step = -1;
}

//update telemetry
void DronePhysics::update()
{
  DroneTelemetry newTelemetry{};


  {
    std::lock_guard<std::mutex> lock(drMutex);  //@updating targets
    telemetry = std::move(newTelemetry);
  }

//  localTimeSec = std::fmod(localTimeSec + tgtTimeStep, trackDuration);
}

auto DronePhysics::getTelemetry() -> DroneTelemetry
{
  std::lock_guard<std::mutex> lock(drMutex);  //@reading target
  return telemetry;
}

}  // namespace drone