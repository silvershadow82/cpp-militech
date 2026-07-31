#pragma once
#include "common.hpp"
#include <string>

class IConfigLoader {
public:
  virtual void load() = 0;
  virtual DroneConfig getConfig() = 0;
  virtual AmmoParams* getAmmoParams() = 0;
  virtual int getAmmoCount() = 0;
  virtual ~IConfigLoader() {};
};

class FileConfigLoader : public IConfigLoader {
private:
  DroneConfig droneConfig;
  AmmoParams* ammoParams;
  bool configAvailable;
  int ammoCount;
  std::string configFileName;
  std::string ammoFileName;
  int readConfig();
  int readAmmoData();

public:
  FileConfigLoader(const std::string& configFileName, const std::string& ammoFileName);
  void load() override;
  AmmoParams* getAmmoParams() override;
  DroneConfig getConfig() override;
  int getAmmoCount() override;
  ~FileConfigLoader() override;
};