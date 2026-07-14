#pragma once
#include "Types.h"
#include "interfaces/IConfigLoader.h"
#include <string>
#include <unordered_map>

class FileConfigLoader : public IConfigLoader {
private:
  bool configAvailable;
  DroneConfig droneConfig;
  std::unordered_map<std::string, AmmoParams> ammoParams;
  std::string configFileName;
  std::string ammoFileName;
  int readConfig();
  int readAmmoData();

public:
  FileConfigLoader(const std::string &configFileName, const std::string &ammoFileName);
  void load() override;
  std::unordered_map<std::string, AmmoParams> getAmmoParams() override;
  DroneConfig getConfig() override;
  int getAmmoCount() override;
  ~FileConfigLoader() override;
};