#pragma once

#include <cstdint>

namespace dto {
enum DroneState : std::uint8_t { STOPPED = 0, ACCELERATING, DECELERATING, TURNING, MOVING };

struct DroneCommand {
  DroneState state = STOPPED;  // новий режим
  float angleSpeed = 0.0F;  // Кутова швидкість повороту Rad/sec
};

/* timeSecSinceStart -  це час останнього оновлення значень фізики.
   Потрібно для того, щоб компенсувати нерівномірність
 кроків, яка виникне при збереженні аутпуту */
struct DroneTelemetry {
  float timeSecSinceStart;  // to become uint32_t t_ms;
  float x, y;               // coordinates
  float z;                  // altitude
  float speed;
  float dir;         // Rad
  DroneState state;  // 0..4/
};

}