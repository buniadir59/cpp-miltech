#include "providers/ThreadSafeTargetProvider.hpp"
#include "dto/Target.hpp"
#include "config/TimeTracker.hpp"
#include "math/point_math.hpp"

#include <cmath>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <fstream>
#include <exception>
#include <thread>
#include <iostream>

using json = nlohmann::json;

/* -sets threadReady;
   -waits for start;
   -calls update() every target time step to provide respective coordinates and velociteis for targets
   -ends on request to stop. */
void ThreadSafeTargetProvider::run() noexcept
{
  try {
    update();  // tracks initial values
    threadReady.store(true);
    while (!threadStart.load() && !threadStopRequested.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (threadStopRequested.load()) {
      return;
    }
    TimeTracker& tt = TimeTracker::getInstance();

    while (!threadStopRequested.load()) {
      update();  // every target time step
      
      localTimeSec = std::fmod(localTimeSec+tgtTimeStep, trackDuration);

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
  std::vector<dto::Target> tgt_now_;
  tgt_now_.reserve(tgtTracks.size());  //=>5
  const auto nSteps = tgtTracks.front().size();
  const auto ind = static_cast<std::size_t>(localTimeSec / arrTimeStep);  // by local time logic, not more than track duration
  const auto nextInd = (ind + 1) % nSteps;
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
}

auto ThreadSafeTargetProvider::getTarget(int idx) -> dto::Target
{
  if (idx < 0 || static_cast<std::size_t>(idx) >= tgtTracks.size()) {
    throw std::runtime_error("Invalid target index");
  }

  std::lock_guard<std::mutex> lock(tgtsMutex);  //@reading target
  return currTgts[idx];
}

auto ThreadSafeTargetProvider::parseJson(const std::string& source) -> std::vector<std::vector<pointmath::Point>>
{
  std::vector<std::vector<pointmath::Point>> tgtTracks;
  std::ifstream json_file(source);

  if (!json_file.is_open()) {
    throw std::runtime_error("Error loading targets");
  }

  try {
    json tgts_j;
    json_file >> tgts_j;

    size_t tgtCount_ = tgts_j["targetCount"];
    size_t nOfTgtTimeSteps_ = tgts_j["timeSteps"];

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
  return tgtTracks;
}
