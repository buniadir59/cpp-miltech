#pragma once

#include "interfaces/IConfigLoader.hpp"

namespace dto {
struct Ammo;
struct MissionConfig;

}  // namespace dto
// читає config.json і ammo.json
class FileConfigLoader : public IConfigLoader {
public:
  auto load(const std::string& conf_source, const std::string& ammo_source) -> bool override;

  auto getConfig() const -> const dto::MissionConfig& override { return config_; };
  auto getAmmoParams() const -> const dto::Ammo& override { return selected_ammo_; };

private:
  void validate_input() const;

  dto::MissionConfig config_{};
  dto::Ammo selected_ammo_{};
};