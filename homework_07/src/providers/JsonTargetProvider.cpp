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

namespace {

} //eo namespace


auto JsonTargetProvider::makeTarget(double currentTimeS,
                                    const pointmath::Point* track) -> dto::Target {
  const double cycle_time = tgtTimeStep * static_cast<double>(nOfTgtTimeSteps);
  const double time_in_cycle = std::fmod(currentTimeS, cycle_time);

  const auto idx = static_cast<std::size_t>(std::floor(time_in_cycle / tgtTimeStep));
  const std::size_t next_idx = (idx + 1) % nOfTgtTimeSteps;

  const double local_time = time_in_cycle - static_cast<double>(idx) * tgtTimeStep;

  const pointmath::Point velocity = (track[next_idx] - track[idx]) / tgtTimeStep;

  return dto::Target{
      .position = track[idx] + velocity * local_time,
      .velocity = velocity,
  };
}


auto JsonTargetProvider::getTarget(int idx) -> dto::Target {
  if (clock == nullptr) {
    throw std::runtime_error("Clock is not available");
  }

  if (idx < 0 || static_cast<std::size_t>(idx) >= tgtCount) {
    throw std::runtime_error("Invalid target index");
  }

  const double currentTimeS = clock->nowS();

  return makeTarget(currentTimeS, tgtTracks[idx]);
}


auto JsonTargetProvider::parseJson(const char* source) -> void {
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

    if ((tgtCount > kMaxTargetCount) || (nOfTgtTimeSteps > kMaxTargetTimeSteps) || (nOfTgtTimeSteps < 2))
    {
      throw std::runtime_error("Invalid parameters in targets json.");
    }

    for (size_t i = 0; i < tgtCount; ++i) {
      for (size_t j = 0; j < nOfTgtTimeSteps; ++j) {
        tgtTracks[i][j].x = tgts_j["targets"][i]["positions"][j]["x"];
        tgtTracks[i][j].y = tgts_j["targets"][i]["positions"][j]["y"];
      }
    }
/*     auto coords = new Point*[tgtCount]; 
      for (size_t i = 0; i < tgtCount; ++i) {
      coords[i] = new Point[nOfTgtTimeSteps];

      for (size_t j = 0; j < nOfTgtTimeSteps; ++j) {
        coords[i][j].x = tgts_j["targets"][i]["positions"][j]["x"];
        coords[i][j].y = tgts_j["targets"][i]["positions"][j]["y"];
      }
    }

    tgtTracks = coords; */
  }
  catch (const std::exception& error) {
    throw std::runtime_error("Error loading targets");
  }
}

}


/* JsonTargetProvider::~JsonTargetProvider() 
{
  if (tgtTracks) {
    for (size_t i = 0; i < tgtCount; ++i) {
      delete[] tgtTracks[i];
    }
    delete[] tgtTracks;
  }
} */