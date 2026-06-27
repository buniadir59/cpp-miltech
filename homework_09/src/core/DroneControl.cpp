#include "core/DroneControl.hpp"

#include <cmath>
#include <memory>

namespace core {

// Оновити координати, швидкість та стан дрона відповідно до поточної кроку
auto DroneControl::execFly() -> void
{
  auto next = state->execute(ctx);
  if (next) {
    state = std::move(next);
  }
}

}  // namespace core