#pragma once

//#include "dto/DropSolution.hpp"
#include "math/point_math.hpp"

namespace dto {

struct SimStep {
  pointmath::Point pos;              // позиція дрона
  float direction;        // напрямок (рад)
  int state;              // стан автомата (0-4)
  int targetIdx;          // індекс поточної цілі
  pointmath::Point dropPoint;        // точка скиду (куди летить дрон)
  pointmath::Point aimPoint;         // куди впаде бомба (якщо скинути зараз)
  pointmath::Point predictedTarget;  // прогнозована позиція цілі
};

}