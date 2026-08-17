#pragma once

#include "Types.h"
#include <string>
#include <unordered_map>

class IConfigLoader {
public:
  virtual void load() = 0;
  virtual DroneConfig getConfig() = 0;
  virtual std::unordered_map<std::string, AmmoParams> getAmmoParams() = 0;
  virtual int getAmmoCount() = 0;
  virtual ~IConfigLoader() = default;
};
