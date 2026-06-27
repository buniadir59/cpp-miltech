#pragma once

#include "dto/Target.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/ISimulationClock.hpp"
#include "math/point_math.hpp"

#include <vector>
#include <string>

// Завантажує таблицю координат цілей з JSON-файлу,
// повертає ціль(координати і швидкість) згідно з поточним часом симуляції
class JsonTargetProvider final : public ITargetProvider {
  const std::string kTgtsFileName = "targets.json";

  static constexpr size_t kMaxTargetCount = 64;
  static constexpr size_t kMaxTargetTimeSteps = 200;

  size_t tgtCount = 0;         // number of active targets
  size_t nOfTgtTimeSteps = 0;  // length of pos array

  std::vector<std::vector<pointmath::Point>> tgtTracks;
  const ISimulationClock* simClock{nullptr};

  auto parseJson(const std::string& source) -> void;
  auto makeTarget(const std::vector<pointmath::Point>& track) -> dto::Target;

public:
  JsonTargetProvider(const std::string& path) { parseJson(path); }

  auto getTargetCount() -> int override { return static_cast<int>(tgtCount); }

  auto getTarget(int idx) -> dto::Target override;

  auto init(const ISimulationClock* clock) -> void override;
};
