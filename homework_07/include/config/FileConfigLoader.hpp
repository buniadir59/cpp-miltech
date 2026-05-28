#pragma once

#include "dto/Ammo.hpp"
#include "dto/MissionConfig.hpp"
#include "interfaces/IConfigLoader.hpp"

//читає config.json і ammo.json
class FileConfigLoader : public IConfigLoader {

 public:
  auto virtual load(const char* source) -> bool override;
  auto virtual getConfig() const -> const dto::MissionConfig& override;
  auto virtual getAmmoParams() const -> const dto::Ammo& override;

  ~FileConfigLoader() override;
    
 private:
 // auto loadConfig(const char* config_path) -> bool;
 // auto loadAmmoTable(const char* ammo_path) -> bool;
 // auto findAmmoByName(const std::string& ammo_name) const -> const dto::Ammo*;

  dto::MissionConfig config_{};
  dto::Ammo selected_ammo_{};
  dto::Ammo* ammoTable_{nullptr};

};