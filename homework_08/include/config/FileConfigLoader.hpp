#pragma once

#include "dto/Ammo.hpp"
#include "dto/MissionConfig.hpp"
#include "interfaces/IConfigLoader.hpp"

// читає config.json і ammo.json
class FileConfigLoader : public IConfigLoader {
public:
  auto load(const char* source) -> bool override;

  auto getConfig() const -> const dto::MissionConfig& override { return config_; };
  auto getAmmoParams() const -> const dto::Ammo& override { return selected_ammo_; };

private:
  void validate_input() const;

  dto::MissionConfig config_{};
  dto::Ammo selected_ammo_{};
};