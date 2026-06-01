#pragma once

#include "common.hpp"
#include "debug.hpp"

#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include <fstream>
#include <iostream>
#include <string>

using json = nlohmann::json;

// Interfaces
class ITargetProvider {
public:
  virtual int getTargetCount() = 0;
  virtual Target getTarget(int index) = 0;
  virtual ~ITargetProvider(){};
};

class IBallisticSolver {
public:
  virtual void init(DroneConfig droneConfig, PayloadParams payloadParams) = 0;
  virtual int solve(SimState& state) = 0;
  virtual ~IBallisticSolver(){};
};

class IConfigLoader {
public:
  virtual void load() = 0;
  virtual DroneConfig getConfig() = 0;
  virtual AmmoParams* getAmmoParams() = 0;
  virtual int getAmmoCount() = 0;
  virtual ~IConfigLoader(){};
};

// Implementations

class AnalyticalSolver : public IBallisticSolver {
private:
  PayloadParams pp;
  DroneConfig droneConfig;

  float payloadTimeOfFlight()
  {
    // Calculate the payload travel time t
    // Using Cardano formula to the cubic equation at^3+bt^2+c = 0
    float altitude = this->droneConfig.altitude;
    float speed = this->droneConfig.attackSpeed;

    float a = static_cast<float>(pp.d * G * pp.m - 2.0 * pp.d2 * pp.l * speed);
    float b = static_cast<float>(-3.0 * G * pp.m2 + 3.0 * pp.d * pp.l * pp.m * speed);
    float c = static_cast<float>(6.0 * pp.m2 * altitude);
    float a2 = a * a, a3 = a2 * a;
    float b2 = b * b, b3 = b2 * b;

    // Cardano trigonometric solution
    float p = static_cast<float>(-b2 / (3.0 * a2));
    float q = static_cast<float>((2.0 * b3) / (27.0 * a3) + c / a);
    float arg = static_cast<float>(((3.0 * q) / (2.0 * p)) * sqrt(-3.0 / p));

    // Check that arg is within the range [-1, 1]
    if (arg < -1 || arg > 1) {
      std::cerr << "Warning: acos arg is out of range: " << arg << std::endl;
      return -1.f;
    }

    float phi = acos(arg);

    // Calculate payload travel time
    float t = static_cast<float>(2.0 * sqrt(-p / 3.0) * cos((phi + 4.0 * M_PI) / 3.0) - b / (3.0 * a));

    return t;
  }

  float calcHDistance(float t)
  {
    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t2 * t2;
    float t5 = t3 * t2;

    float speed = this->droneConfig.attackSpeed;
    float e1 = t * speed;
    float e2 = static_cast<float>(-t2 * pp.d * speed / (2.0 * pp.m));
    float e3 = static_cast<float>(t3 * (6.0 * pp.d * G * pp.l * pp.m - 6.0 * pp.d2 * (pp.l2 - 1.0) * speed) / (36.0 * pp.m2));
    float e4 = static_cast<float>(t4 *
                                  (3.0 * pp.d3 * (pp.l2 + 1.0) * pp.l2 * speed + 6.0 * pp.d3 * (pp.l2 + 1.0) * pp.l4 * speed -
                                   6.0 * pp.d2 * G * (pp.l4 + pp.l2 + 1.0) * pp.l * pp.m) /
                                  (36.0 * pow((pp.l2 + 1.0), 2) * pp.m3));
    float e5 = static_cast<float>(t5 * (3.0 * pp.d3 * G * pp.l3 * pp.m - 3.0 * pp.d4 * pp.l2 * (pp.l2 + 1.0) * speed) /
                                  (36.0 * (pp.l2 + 1.0) * pp.m4));

    // Calculate the payload travel distance h
    float h = e1 + e2 + e3 + e4 + e5;

    return h;
  }

public:
  AnalyticalSolver(){};

  void init(DroneConfig droneConfig, PayloadParams payloadParams) override
  {
    this->droneConfig = droneConfig;
    this->pp = payloadParams;
  }
  int solve(SimState& state) override
  {
    float t = payloadTimeOfFlight();

    if (t <= 0) {
      std::cout << "Invalid t=" << t << std::endl;
      return 1;
    }

    float h = calcHDistance(t);

    if (h <= 0) {
      std::cout << "Invalid h=" << h << std::endl;
      return 1;
    }

    // Calculate drone to target distance
    float distanceToTarget = Coord::distance(state.dronePos, state.targetPos);

    if (distanceToTarget <= 0) {
      std::cout << "Invalid D=" << distanceToTarget << std::endl;
      return 1;
    }
    // Check if drone has to maneuvre and calculate new xd, yd

    if (h + droneConfig.accelPath > distanceToTarget) {
      if (fabs(distanceToTarget) < 1e-6) {
        state.dronePos.x = state.targetPos.x - (h + droneConfig.accelPath);
        state.dronePos.y = state.targetPos.y;
        // outDronePos.x = targetPos.x - (h + accelerationPath);

        // DEBUG("with intermediate point: NewDronePos=(" << state.dronePos.x << "," << state.dronePos.y << ")");

        distanceToTarget = h + droneConfig.accelPath;
      }
      else {
        state.dronePos = state.targetPos - (state.targetPos - state.dronePos) * (h + droneConfig.accelPath) / distanceToTarget;

        // DEBUG("NewDronePos=(" << state.dronePos.x << ", " << state.dronePos.y << ")");
        // xd = targetX - (targetX - xd) * (h + accelerationPath) / distanceToTarget;
        // yd = targetY - (targetY - yd) * (h + accelerationPath) / distanceToTarget;
        distanceToTarget = Coord::distance(state.dronePos, state.targetPos);
      }
    }

    float ratio = (distanceToTarget - h) / distanceToTarget;

    // Calculate drop point coordinates
    Coord dir = {static_cast<float>(cos(state.droneAngle)), static_cast<float>(sin(state.droneAngle))};
    state.dropPoint = state.dronePos + (state.targetPos - state.dronePos) * ratio;
    state.aimPoint = state.dronePos + dir * h;
    state.payloadDropTime = t;

    // fireX = xd + (targetX - xd) * ratio;
    // fireY = yd + (targetY - yd) * ratio;

    return 0;
  }
};

class FileConfigLoader : public IConfigLoader {
private:
  DroneConfig droneConfig;
  AmmoParams* ammoParams;
  bool configAvailable;
  int ammoCount;
  std::string configFileName;
  std::string ammoFileName;

  int readConfig()
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

    std::strncpy(this->droneConfig.ammoName, jc["ammo"].get<std::string>().c_str(), 31);

    this->droneConfig.simTimeStep = jc["simulation"]["timeStep"];
    this->droneConfig.hitRadius = jc["simulation"]["hitRadius"];
    this->droneConfig.arrayTimeStep = jc["targetArrayTimeStep"];

    configFile.close();

    return 0;
  }

  int readAmmoData()
  {
    std::ifstream ammoFile(this->ammoFileName);
    if (!ammoFile.is_open()) {
      std::cerr << "Unable to open ammo file!" << this->ammoFileName << std::endl;
      return 1;
    }
    json ja;
    ammoFile >> ja;

    this->ammoCount = static_cast<size_t>(ja.size());
    this->ammoParams = new AmmoParams[ammoCount];

    for (int i = 0; i < ammoCount; i++) {
      this->ammoParams[i] = AmmoParams{};
      this->ammoParams[i].mass = ja[i]["mass"];
      this->ammoParams[i].drag = ja[i]["drag"];
      this->ammoParams[i].lift = ja[i]["lift"];
      std::strncpy(this->ammoParams[i].name, ja[i]["name"].get<std::string>().c_str(), MAX_AMMO_NAME - 1);
    }

    ammoFile.close();
    return 0;
  }

public:
  FileConfigLoader(const std::string& configFileName, const std::string& ammoFileName)
    : configFileName(configFileName)
    , ammoFileName(ammoFileName){};

  void load() override
  {
    int ret_code = this->readConfig();
    if (ret_code == 0) {
      ret_code = this->readAmmoData();
    }
    this->configAvailable = ret_code == 0;
  };

  AmmoParams* getAmmoParams() override
  {
    if (this->configAvailable) {
      return this->ammoParams;
    }
    return nullptr;
  };

  DroneConfig getConfig() override
  {
    if (!this->configAvailable) {
      this->load();
    }
    return this->droneConfig;
  }

  int getAmmoCount() override
  {
    if (!this->configAvailable) {
      this->load();
    }
    return this->ammoCount;
  }

  ~FileConfigLoader() override { delete[] this->ammoParams; }
};

class JsonTargetProvider : public ITargetProvider {
private:
  const std::string jsonFileName;
  int targetCount{0};
  Target* targets;

  int readTargetData()
  {
    std::ifstream targetFile(this->jsonFileName);
    if (!targetFile.is_open()) {
      std::cerr << "Unable to open targets.json!" << std::endl;
      return 1;
    }
    json jt;
    targetFile >> jt;

    int timeStepCount = jt["timeSteps"];
    this->targetCount = jt["targetCount"];
    this->targets = new Target[targetCount];

    for (int i = 0; i < targetCount; i++) {
      this->targets[i] = Target(i, timeStepCount);
      for (int j = 0; j < timeStepCount; j++) {
        this->targets[i].setPosAt(j, jt["targets"][i]["positions"][j]["x"], jt["targets"][i]["positions"][j]["y"]);
      }
    }
    targetFile.close();

    LOG("read targetCount=" << this->targetCount);
    LOG("read timeSteps=" << timeStepCount);

    return 0;
  }

public:
  JsonTargetProvider(const std::string& jsonFileName)
    : jsonFileName(jsonFileName)
  {
    this->readTargetData();
  }

  int getTargetCount() override { return this->targetCount; }

  Target getTarget(int index) override
  {
    if (index < 0 || index >= this->targetCount) {
      return Target();
    }
    return this->targets[index];
  }

  ~JsonTargetProvider() override
  {
    if (this->targets != nullptr) {
      delete[] this->targets;
    }
  }
};