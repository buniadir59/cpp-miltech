#pragma once

#include "dto/Target.hpp"
#include "dto/MissionConfig.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "math/point_math.hpp"

#include <cmath>
#include <vector>
#include <string>
#include <atomic>

// Завантажує таблицю координат цілей з JSON-файлу,
// повертає ціль(координати і швидкість) згідно з поточним часом симуляції
class ThreadSafeTargetProvider final : public ITargetProvider {
  static constexpr int kMaxTargetCount = 64;
  static constexpr int kMaxTargetTimeSteps = 200;

  std::vector<std::vector<pointmath::Point>> tgtTracks;
  std::vector<dto::Target> currTgts;  // current positions and velocities of simulated targets

  double arrTimeStep;
  double tgtTimeStep;

  double localTimeSec = 0.0;  // local time for target thread
  double trackDuration = 0.0;

  mutable std::mutex tgtsMutex;
  std::atomic<bool> threadStopRequested{false};
  std::atomic<bool> threadStart{false};
  std::atomic<bool> threadReady{false};
  std::atomic<bool> failed{false};

  auto parseJson(const std::string& source) -> void;  // called when created
  auto update() -> void;                              // double time

public:
  ThreadSafeTargetProvider(const dto::MissionConfig& config, const std::string& path)
    : arrTimeStep(config.targetArrayTimeStep)
    , tgtTimeStep(config.targetTimeStep)
  {
    parseJson(path);
    trackDuration = static_cast<double>(tgtTracks.size()) * arrTimeStep;
  }

  [[nodiscard]] auto getTargetCount() -> int override { return static_cast<int>(tgtTracks.size()); }
  [[nodiscard]] auto getTarget(int idx) -> dto::Target override;

  auto start() -> void override { threadStart.store(true); };
  auto stop() -> void override { threadStopRequested.store(true); };
  [[nodiscard]] auto isThreadReady() -> bool override { return threadReady.load(); };
  [[nodiscard]] auto hasFailed() const -> bool override { return failed.load(); };
  auto run() noexcept -> void override;
};
