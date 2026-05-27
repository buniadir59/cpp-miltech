// include/interfaces/i_config_loader.hpp
#pragma once

#include "dto/Ammo.hpp"
#include "dto/MissionConfig.hpp"



class IConfigLoader {
 public:
  virtual auto load(const char* source) -> bool = 0;
  virtual auto getConfig() const -> const dto::MissionConfig& = 0;
  virtual auto getAmmoParams() const -> const dto::Ammo& = 0;

  virtual ~IConfigLoader() = default;
};

//namespace interfaces {}  // namespace interfaces