#pragma once

#include "math/point_math.hpp"


namespace dto {

  struct Target {
    pointmath::Point position{};
    pointmath::Point velocity{};
  };

}

