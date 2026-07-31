#include "config/FileConfigLoader.h"
#include "Types.h"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

using json = nlohmann::json;

int FileConfigLoader::readConfig()
{
  std::ifstream configFile(this->configFileName);
  if (!configFile.is_open()) {
    std::cerr << "Unable to open config file" << this->configFileName << std::endl;
    return 1;
  }
  json jc;
  configFile >> jc;

  this->droneConfig = DroneConfig{};

  // Work with json
  this->droneConfig.startPos.x = jc["drone"]["position"]["x"];
  this->droneConfig.startPos.y = jc["drone"]["position"]["y"];
  this->droneConfig.altitude = jc["drone"]["altitude"];
  this->droneConfig.initialDir = jc["drone"]["initialDirection"];
  this->droneConfig.attackSpeed = jc["drone"]["attackSpeed"];
  this->droneConfig.accelPath = jc["drone"]["accelerationPath"];
  this->droneConfig.angularSpeed = jc["drone"]["angularSpeed"];
  this->droneConfig.turnThreshold = jc["drone"]["turnThreshold"];
  this->droneConfig.ammoName = jc["ammo"];

  // std::strncpy(this->droneConfig.ammoName,
  // jc["ammo"].get<std::string>().c_str(), 31);

  this->droneConfig.simTimeStep = jc["simulation"]["timeStep"];
  this->droneConfig.hitRadius = jc["simulation"]["hitRadius"];
  this->droneConfig.arrayTimeStep = jc["targetArrayTimeStep"];

  configFile.close();

  return 0;
}

int FileConfigLoader::readAmmoData()
{
  std::ifstream ammoFile(this->ammoFileName);
  if (!ammoFile.is_open()) {
    std::cerr << "Unable to open ammo file!" << this->ammoFileName << std::endl;
    return 1;
  }
  json ja;
  ammoFile >> ja;

  int ammoCount = static_cast<size_t>(ja.size());
  this->ammoParams = std::unordered_map<std::string, AmmoParams>(ammoCount);

  for (int i = 0; i < ammoCount; i++) {
    std::string name = ja[i]["name"];
    this->ammoParams.emplace(name, AmmoParams{.name = name, .mass = ja[i]["mass"], .drag = ja[i]["drag"], .lift = ja[i]["lift"]});
    // this->ammoParams[i] = AmmoParams{};
    // this->ammoParams[i].mass = ja[i]["mass"];
    // this->ammoParams[i].drag = ja[i]["drag"];
    // this->ammoParams[i].lift = ja[i]["lift"];
    // std::strncpy(this->ammoParams[i].name,
    // ja[i]["name"].get<std::string>().c_str(), MAX_AMMO_NAME - 1);
  }

  ammoFile.close();
  return 0;
}

FileConfigLoader::FileConfigLoader(const std::string &configFileName, const std::string &ammoFileName)
  : configFileName(configFileName)
  , ammoFileName(ammoFileName) {};

void FileConfigLoader::load()
{
  int ret_code = this->readConfig();
  if (ret_code == 0) {
    ret_code = this->readAmmoData();
  }
  this->configAvailable = ret_code == 0;
};

std::unordered_map<std::string, AmmoParams> FileConfigLoader::getAmmoParams()
{
  if (!this->configAvailable) {
    this->load();
  }
  return this->ammoParams;
};

DroneConfig FileConfigLoader::getConfig()
{
  if (!this->configAvailable) {
    this->load();
  }
  return this->droneConfig;
}

int FileConfigLoader::getAmmoCount()
{
  if (!this->configAvailable) {
    this->load();
  }
  return this->ammoParams.size();
}

FileConfigLoader::~FileConfigLoader() {}