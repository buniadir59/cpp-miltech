#include "providers/ThreadSafeTargetProvider.hpp"
#include "dto/Target.hpp"
#include "core_/TimeTracker.hpp"
#include "math/point_math.hpp"

#include <cmath>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <fstream>
#include <exception>
#include <stdexcept>
#include <thread>
#include <iostream>

using json = nlohmann::json;

/* -sets threadReady;
   -waits for start;
   -calls update() every arrayTimeStep / timeS cale;
   -ends on request to stop. */
void ThreadSafeTargetProvider::run() noexcept
{
  try {
    update();  // initial values
    threadReady.store(true);
    while (!threadStart.load() && !threadStopRequested.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (threadStopRequested.load()) {
      return;
    }
    TimeTracker& tt = TimeTracker::getInstance();

    while (!threadStopRequested.load()) {
      update();
      auto wakeup = tt.nextWakeup(tgtTimeStep);
      std::this_thread::sleep_until(wakeup);
    }
  }
  catch (const std::exception& error) {
    std::cerr << "Provider thread error: " << error.what() << '\n';
    failed.store(true);
    threadStopRequested.store(true);
  }
  catch (...) {
    std::cerr << "Provider thread unknown error\n";
    failed.store(true);
    threadStopRequested.store(true);
  }
}

void ThreadSafeTargetProvider::update()
{
  const auto nTgts_ = tgtTracks.size();
  std::vector<dto::Target> tgt_now_;
  tgt_now_.reserve(nTgts_);

  const auto ind = static_cast<std::size_t>(localTimeSec / arrTimeStep);

  const auto nextInd = (ind + 1) % nTgts_;
  const double timeInsideSegment = localTimeSec - static_cast<double>(ind) * arrTimeStep;

  for (const auto& track : tgtTracks) {
    const auto& from = track[ind];
    const auto& to = track[nextInd];

    const auto velocity = (to - from) / arrTimeStep;
    tgt_now_.push_back(dto::Target{
      .position = from + velocity * timeInsideSegment,
      .velocity = velocity,
    });
  }

  {
    std::lock_guard<std::mutex> lock(tgtsMutex);  //@updating targets
    currTgts = std::move(tgt_now_);
  }

  localTimeSec = std::fmod(localTimeSec + tgtTimeStep, trackDuration);
}

auto ThreadSafeTargetProvider::getTarget(int idx) -> dto::Target
{
  if (idx < 0 || static_cast<std::size_t>(idx) >= tgtTracks.size()) {
    throw std::runtime_error("Invalid target index");
  }

  std::lock_guard<std::mutex> lock(tgtsMutex);  //@reading target
  return currTgts[idx];
}

auto ThreadSafeTargetProvider::parseJson(const std::string& source) -> void
{
  std::ifstream json_file(source);

  if (!json_file.is_open()) {
    throw std::runtime_error("Error loading targets");
  }

  try {
    json tgts_j;
    json_file >> tgts_j;

    int tgtCount_ = tgts_j["targetCount"];
    int nOfTgtTimeSteps_ = tgts_j["timeSteps"];

    // validate target params
    if ((tgtCount_ > kMaxTargetCount) || (nOfTgtTimeSteps_ > kMaxTargetTimeSteps) || (nOfTgtTimeSteps_ < 2)) {
      throw std::runtime_error("Invalid parameters in targets json.");
    }

    tgtTracks.clear();
    tgtTracks.reserve(tgtCount_);

    for (const auto& target_json : tgts_j["targets"]) {
      std::vector<pointmath::Point> track;
      track.reserve(nOfTgtTimeSteps_);

      for (const auto& pos_json : target_json["positions"]) {
        track.push_back(pointmath::Point{
          pos_json["x"].get<double>(),
          pos_json["y"].get<double>(),
        });
      }

      tgtTracks.push_back(std::move(track));
    }
  }

  catch (const std::exception& error) {
    throw std::runtime_error("Error loading targets");
  }
}
