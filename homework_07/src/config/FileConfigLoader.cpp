// include/loaders/file_config_loader.hpp
#pragma once

#include <string>
#include <vector>

#include "dto/Ammo.hpp"
#include "dto/MissionConfig.hpp"
#include "interfaces/IConfigLoader.hpp"



class FileConfigLoader : public IConfigLoader {
 public:
  auto virtual load(const char* source) -> bool override;
  auto virtual getConfig() const -> const dto::MissionConfig& override;
  auto virtual getAmmoParams() const -> const dto::Ammo& override;

 private:
  auto loadConfig(const std::string& config_path) -> bool;
  auto loadAmmoTable(const std::string& ammo_path) -> bool;
  auto findAmmoByName(const std::string& ammo_name) const -> const dto::Ammo*;

  dto::MissionConfig config_{};
  dto::Ammo selected_ammo_{};
  std::vector<dto::Ammo> ammo_table_{};
};


//namespace loaders {}  // namespace loaders 