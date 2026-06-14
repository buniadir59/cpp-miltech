#include "config/FileConfigLoader.hpp"
#include "dto/Ammo.hpp"
#include "dto/MissionConfig.hpp"

#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <exception>
#include <string>
#include <unordered_map>

using json = nlohmann::json;

namespace {
const char* const kAmmosFileName = "ammo.json";
const char* const kInputFileName = "config.json";

}  // namespace

auto FileConfigLoader::validate_input() const -> void
{
  if ((config_.attack_speed < 0.0) || (config_.turn_threshold < 0)) {
    throw std::invalid_argument("Drone attack speed and turn threshold must not be negative");
  }

  if ((config_.acceleration_path <= 0.0) || (config_.altitude <= 0.0) || (config_.angular_speed <= 0.0) || (config_.hit_rad <= 0.0)) {
    throw std::invalid_argument("Drone altitude, acceleration path, angular speed and hit radius must be positive");
  }

  if ((config_.time_step < 0.01) || (config_.tgt_time_step < config_.time_step)) {
    throw std::invalid_argument("Invalid time step and/or target time step value");
  }
}

auto FileConfigLoader::load(const char* source) -> bool
{
  // first, read input.json
  std::filesystem::path full_path = std::filesystem::path(source) / kInputFileName;
  std::ifstream json_file(full_path);

  if (!json_file.is_open()) {
    std::cerr << "Unable to open: " << full_path << '\n';
    return false;
  }

  std::string ammo_name;

  try {
    json jsn;
    json_file >> jsn;

    config_.drone_position = {jsn["drone"]["position"]["x"], jsn["drone"]["position"]["y"]};
    config_.altitude = jsn["drone"]["altitude"];
    config_.initial_direction = jsn["drone"]["initialDirection"];
    config_.attack_speed = jsn["drone"]["attackSpeed"];
    config_.acceleration_path = jsn["drone"]["accelerationPath"];
    config_.angular_speed = jsn["drone"]["angularSpeed"];
    config_.turn_threshold = jsn["drone"]["turnThreshold"];

    config_.hit_rad = jsn["simulation"]["hitRadius"];
    config_.time_step = jsn["simulation"]["timeStep"];

    config_.tgt_time_step = jsn["targetArrayTimeStep"];

    ammo_name = jsn["ammo"].get<std::string>();

    validate_input();
  }
  catch (const std::exception& error) {
    std::cerr << "Invalid or incomplete data in " << full_path << '\n';
    return false;
  }

  // second, read ammo.json
  full_path = std::filesystem::path(source) / kAmmosFileName;
  std::ifstream json_ammo_file(full_path);

  if (!json_ammo_file.is_open()) {
    std::cerr << "Unable to open: " << full_path << '\n';
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
    std::cerr << "Invalid or incomplete data in " << full_path << '\n';
    return false;
  }

  return true;
}
