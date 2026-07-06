#include "config/FileConfigLoader.hpp"
#include "dto/Ammo.hpp"
#include "dto/MissionConfig.hpp"

#include <nlohmann/json.hpp>
#include <fstream>
#include <exception>
#include <string>
#include <unordered_map>
#include <iostream>

using json = nlohmann::json;

namespace {
inline constexpr double kEps = 1e-9;
inline constexpr double kMinStepS = 0.005;
inline constexpr double kMinAltitude = 40;  // for now reasonable limit vased on ballistic table
}  // namespace

/*
Нові параметри — лише два, обидва в секції simulation:
"simulation": {
        "targetTimeStep": 0.05,
        "physicsTimeStep": 0.01,
        "timeScale": 10.0
}
•       physicsTimeStep — крок оновлення фізики дрона, окремий від simTimeStep.
•       timeScale — прискорення часу: потік інтегрує крок dt, а спить dt / timeScale реального часу.
•       Період оновлення цілей — наявний arrayTimeStep, період кроку MissionProcessor — наявний simTimeStep. Нових параметрів для них немає.
Якщо параметрів немає у файлі - використовувати дефолт
 */

auto FileConfigLoader::validate_input() const -> void
{
  if ((config_.attackSpeed < 0.0) || (config_.turnThreshold < 0)) {
    throw std::invalid_argument("Drone attack speed and turn threshold must not be negative");
  }

  if ((config_.kAccelerationPath <= kEps) || (config_.kAltitude < kMinAltitude) || (config_.maxAngularSpeedRadPerS <= kEps) ||
      (config_.hitRadius <= kEps) || (config_.timeScale <= kMinStepS) || (config_.targetTimeStep <= kMinStepS) ||
      (config_.targetArrayTimeStep <= kMinStepS) || (config_.physicsTimeStep <= kMinStepS)) {
    throw std::invalid_argument("Drone altitude, acceleration path, angular speed and hit radius must be positive");
  }

  // physics step must be < simulation step, tgt array step must be greater than simulation step
  if ((config_.timeStep < 0.01) || (config_.targetArrayTimeStep < config_.timeStep) || (config_.physicsTimeStep >= config_.timeStep)) {
    throw std::invalid_argument("Invalid time step and/or target time step value");
  }
}

auto FileConfigLoader::load(const std::string& conf_source, const std::string& ammo_source) -> bool
{
  // first, read input.json
  std::ifstream json_file(conf_source);

  if (!json_file.is_open()) {
    std::cerr << "Unable to open: " << conf_source << '\n';
    return false;
  }

  std::string ammo_name;

  try {
    json jsn;
    json_file >> jsn;

    config_.initialPosition = {jsn["drone"]["position"]["x"], jsn["drone"]["position"]["y"]};
    config_.kAltitude = jsn["drone"]["altitude"];
    config_.initialDirection = jsn["drone"]["initialDirection"];
    config_.attackSpeed = jsn["drone"]["attackSpeed"];
    config_.kAccelerationPath = jsn["drone"]["accelerationPath"];
    config_.maxAngularSpeedRadPerS = jsn["drone"]["angularSpeed"];
    config_.turnThreshold = jsn["drone"]["turnThreshold"];

    config_.hitRadius = jsn["simulation"]["hitRadius"];
    config_.timeStep = jsn["simulation"]["timeStep"];

    config_.targetTimeStep = jsn.value("/simulation/targetTimeStep"_json_pointer, 0.01);
    config_.physicsTimeStep = jsn.value("/simulation/physicsTimeStep"_json_pointer, 0.01);
    config_.timeScale = jsn.value("/simulation/timeScale"_json_pointer, 1.0);

    config_.targetArrayTimeStep = jsn["targetArrayTimeStep"];

    ammo_name = jsn["ammo"].get<std::string>();

    validate_input();
  }
  catch (const std::exception& error) {
    std::cerr << "Invalid or incomplete data in " << conf_source << '\n';
    return false;
  }

  // second, read ammo.json
  std::ifstream json_ammo_file(ammo_source);

  if (!json_ammo_file.is_open()) {
    std::cerr << "Unable to open: " << ammo_source << '\n';
    return false;
  }

  try {
    json ammos;
    json_ammo_file >> ammos;

    size_t nAmmos = ammos.size();
    config_.nAmmos = nAmmos;

    std::unordered_map<std::string, dto::Ammo> ammoTable_;
    for (const auto& ammo_json : ammos) {
      dto::Ammo ammo{
        .name = ammo_json["name"].get<std::string>(),
        .mass = ammo_json["mass"],
        .drag = ammo_json["drag"],
        .lift = ammo_json["lift"],
      };

      ammoTable_.emplace(ammo.name, ammo);
    }

    const auto it = ammoTable_.find(ammo_name);
    if (it == ammoTable_.end()) {
      throw std::runtime_error("Ammo not found");
    }

    selected_ammo_ = it->second;

    return true;
  }

  catch (const std::exception& error) {
    std::cerr << "Invalid or incomplete data in " << conf_source << '\n';
    return false;
  }

  return true;
}
