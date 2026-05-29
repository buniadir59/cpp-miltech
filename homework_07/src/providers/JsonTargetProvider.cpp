#include "providers/JsonTargetProvider.hpp"

#include "dto/Target.hpp"
#include "math/point_math.hpp"

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

auto JsonTargetProvider::makeTarget(size_t timeIdx, const pointmath::Point* track) -> dto::Target
{
  size_t prevInd = timeIdx == 0 ? nOfTgtTimeSteps - 1 : timeIdx - 1;
  return dto::Target {.position = track[timeIdx], 
          .velocity = (track[timeIdx] - track[prevInd]) / tgtTimeStep};

}

auto JsonTargetProvider::getTarget(int idx) -> dto::Target {
  if (clock == nullptr) {
    throw std::runtime_error("Clock is not available");
  }

  if (idx < 0 || static_cast<std::size_t>(idx) >= tgtCount) {
    throw std::runtime_error("Invalid target index");
  }

  const double currentTimeS = clock->nowS();

  const size_t timeIdx = static_cast<std::size_t>(floor(currentTimeS / tgtTimeStep)) 
                                                  % nOfTgtTimeSteps;

  return makeTarget(timeIdx, tgtTracks[idx]);
}


auto JsonTargetProvider::parseJson(const char* source) -> void{
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

    if ((tgtCount > kMaxTargetCount) || (nOfTgtTimeSteps > kMaxTargetTimeSteps))
    {
      throw std::runtime_error("Invalid parameters in targets json.");
    }
    
    auto coords = new Point*[tgtCount]; 
      for (size_t i = 0; i < tgtCount; ++i) {
      coords[i] = new Point[nOfTgtTimeSteps];

      for (size_t j = 0; j < nOfTgtTimeSteps; ++j) {
        coords[i][j].x = tgts_j["targets"][i]["positions"][j]["x"];
        coords[i][j].y = tgts_j["targets"][i]["positions"][j]["y"];
      }
    }

    tgtTracks = coords;
  }
  catch (const std::exception& error) {
    throw std::runtime_error("Error loading targets");
  }
}

}


JsonTargetProvider::~JsonTargetProvider() 
{
  if (tgtTracks) {
    for (size_t i = 0; i < tgtCount; ++i) {
      delete[] tgtTracks[i];
    }
    delete[] tgtTracks;
  }
}