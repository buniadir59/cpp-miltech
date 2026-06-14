#include "providers/JsonTargetProvider.hpp"
#include "dto/Target.hpp"
#include "math/point_math.hpp"

#include <cmath>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <exception>
#include <stdexcept>

using json = nlohmann::json;
using Point = pointmath::Point;

auto JsonTargetProvider::makeTarget(const std::vector<pointmath::Point>& track) -> dto::Target
{
  if (simClock == nullptr) {
    throw std::runtime_error("Clock is not available");
  }

  const double tgtTime = simClock->nowForTargetProvider();
  const double tgtTimeFloor = std::floor(tgtTime);

  const auto idx = (static_cast<std::size_t>(tgtTimeFloor)) % nOfTgtTimeSteps;
  const std::size_t next_idx = (idx + 1) % nOfTgtTimeSteps;

  const double d_time = tgtTime - tgtTimeFloor;

  const pointmath::Point delta = (track[next_idx] - track[idx]);

  return dto::Target{
    .position = track[idx] + delta * d_time,
    .delta = delta,
  };
}

auto JsonTargetProvider::init(const ISimulationClock* clock) -> void
{
  simClock = clock;
}

auto JsonTargetProvider::getTarget(int idx) -> dto::Target
{
  if (idx < 0 || static_cast<std::size_t>(idx) >= tgtCount) {
    throw std::runtime_error("Invalid target index");
  }
  return makeTarget(tgtTracks.at(static_cast<std::size_t>(idx)));
}

auto JsonTargetProvider::parseJson(const std::string&  source) -> void
{
  {
    std::filesystem::path full_path = std::filesystem::path(source) / kTgtsFileName;
    std::ifstream json_file(full_path);

    if (!json_file.is_open()) {
      std::cerr << "Unable to open: " << full_path << '\n';
      throw std::runtime_error("Error loading targets");
    }

    try {
      json tgts_j;
      json_file >> tgts_j;

      tgtCount = tgts_j["targetCount"];
      nOfTgtTimeSteps = tgts_j["timeSteps"];

      if ((tgtCount > kMaxTargetCount) || (nOfTgtTimeSteps > kMaxTargetTimeSteps) || (nOfTgtTimeSteps < 2)) {
        throw std::runtime_error("Invalid parameters in targets json.");
      }

      tgtTracks.clear();
      tgtTracks.reserve(tgtCount);

      for (const auto& target_json : tgts_j["targets"]) {
        std::vector<pointmath::Point> track;
        track.reserve(nOfTgtTimeSteps);

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
}
