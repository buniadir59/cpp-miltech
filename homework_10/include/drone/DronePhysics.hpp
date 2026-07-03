#pragma once

#include "dto/MissionConfig.hpp"

#include "math/point_math.hpp"

#include <cmath>
// #include <vector>
// #include <string>
#include <atomic>
#include <cstdint>

/* TODO
клас фізики дрона . Володіє станом дрона, приймає команди через чергу і у власному потоці кожні physicsTimeStep інтегрує рух.
Based on DroneControl + DroneContext
+ drone states (???)
 */

namespace drone {

// при нульовій швидкості напрямок визначити неможливо - швидкість відображати напрямком та модулем TODO!!!!!!
struct DroneTelemetry {
  uint32_t t_ms;  // timeSecSinceStart;
  float x, y;     // coordinates
  float z;        // altitude
  float vx, vy;   // velocity by coordinates
  float speed;
  float dir;      // Rad
  uint8_t state;  // 0..4/
};

enum DroneState : std::uint8_t { STOPPED = 0, ACCELERATING, DECELERATING, TURNING, MOVING };

struct DroneCommand {  // TODO ? why not destination point and angle at destination???
  DroneState state;    // новий режим
  float angleSpeed;    // Кутова швидкість повороту ????
};

class DronePhysics {
  DroneState currentState = STOPPED;
  double accelerationPath;
  double altitude;
  double attackSpeed;
  double angularSpeed;
  double initialDirection;
  pointmath::Point initialPosition;
  double turnThreshold;
  double physicsTimeStep;  // 0.01

  double timeSecSinceStart = 0.0; /* це час останнього оновлення значень фізики.
  Потрібно для того, щоб компенсувати нерівномірність
кроків, яка виникне при збереженні аутпуту */

  mutable std::mutex drMutex;
  std::atomic<bool> threadStopRequested{false};
  std::atomic<bool> threadStart{false};
  std::atomic<bool> threadReady{false};
  DroneTelemetry telemetry;

  auto update() -> void;

public:
  DronePhysics(const dto::MissionConfig& config)
    : accelerationPath(config.accelerationPath)
    , altitude(config.altitude)
    , attackSpeed(config.attackSpeed)
    , angularSpeed(config.angularSpeed)
    , initialDirection(config.initialDirection)
    , initialPosition(config.initialPosition)
    , turnThreshold(config.turnThreshold)
    , physicsTimeStep(config.physicsTimeStep)
  {
    telemetry = {0, float(initialPosition.x), float(initialPosition.y), float(config.altitude), 0.0, 0.0, 0.0, 0.0, 0};
  }

  auto start() -> void { threadStart = true; };
  auto stop() -> void { threadStopRequested = true; };
  auto isThreadReady() -> bool { return threadReady; };
  auto run() -> void;

  auto sendCommand(DroneCommand) -> void;
  auto getTelemetry() -> DroneTelemetry;

  auto stateToStr(unsigned int state_num) -> const char*
  {
    switch (state_num) {
      case STOPPED:
        return "STOPPED";
      case ACCELERATING:
        return "ACCELERATING";
      case DECELERATING:
        return "DECELERATING";
      case MOVING:
        return "MOVING";
      case TURNING:
        return "TURNING";
    }
    return "UNKNWN";
  }
};

}  // namespace drone