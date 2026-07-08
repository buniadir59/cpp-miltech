#pragma once

#include "dto/Target.hpp"
#include "dto/MissionConfig.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "math/point_math.hpp"

#include <cmath>
#include <vector>
#include <string>
#include <atomic>
#include <stdexcept>

// Завантажує таблицю координат цілей з JSON-файлу,
// повертає ціль(координати і швидкість) згідно з поточним часом симуляції
class ThreadSafeTargetProvider final : public ITargetProvider {
  static constexpr size_t kMinTargetSteps = 2;
  static constexpr size_t kMaxTargetCount = 64;
  static constexpr size_t kMaxTargetTimeSteps = 200;

  const double arrTimeStep;  //=>5s
  const double tgtTimeStep;  //=>0.05s
  std::vector<std::vector<pointmath::Point>> tgtTracks;
  const double trackDuration;

  double localTimeSec = 0.0;  // local time for target thread (0..track duration)

  mutable std::mutex tgtsMutex;
  std::vector<dto::Target> currTgts;  // current positions and velocities of simulated targets

  std::atomic<bool> threadStopRequested{false};
  std::atomic<bool> threadStart{false};
  std::atomic<bool> threadReady{false};
  std::atomic<bool> failed{false};

  auto parseJson(const std::string& source) -> std::vector<std::vector<pointmath::Point>>;  // void;  // called when created
  auto update() -> void;                                                                    // double time

public:
  ThreadSafeTargetProvider(const dto::MissionConfig& config, const std::string& path)
    : arrTimeStep(config.targetArrayTimeStep)
    , tgtTimeStep(config.targetTimeStep)
    , tgtTracks(parseJson(path))
    , trackDuration(static_cast<double>(tgtTracks.front().size()) * arrTimeStep)

  {
    if (tgtTracks.empty() || (tgtTracks.front().size() < kMinTargetSteps)) {
      throw std::runtime_error("Error loading targets");
    }
  }

  [[nodiscard]] auto getTargetCount() -> int override { return static_cast<int>(tgtTracks.size()); }
  [[nodiscard]] auto getTarget(int idx) -> dto::Target override;

  auto start() -> void override { threadStart.store(true); };
  auto stop() -> void override { threadStopRequested.store(true); };
  [[nodiscard]] auto isThreadReady() -> bool override { return threadReady.load(); };
  [[nodiscard]] auto hasFailed() const -> bool override { return failed.load(); };
  auto run() noexcept -> void override;
};
