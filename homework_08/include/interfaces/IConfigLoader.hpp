#pragma once

#include "dto/Ammo.hpp"
#include "dto/MissionConfig.hpp"

#include <string>

// Завантажувач даних: конфіг місії та параметри боєприпасу

class IConfigLoader {
public:
  virtual auto load(const std::string& source) -> bool = 0;
  [[nodiscard]] virtual auto getConfig() const -> const dto::MissionConfig& = 0;
  [[nodiscard]] virtual auto getAmmoParams() const -> const dto::Ammo& = 0;

  virtual ~IConfigLoader() = default;
};
