#include "debug.hpp"
#include "ITargetProvider.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include <iostream>
#include <fstream>

using json = nlohmann::json;

int JsonTargetProvider::readTargetData()
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

JsonTargetProvider::JsonTargetProvider(const std::string& jsonFileName)
  : jsonFileName(jsonFileName)
{
  this->readTargetData();
}

int JsonTargetProvider::getTargetCount()
{
  return this->targetCount;
}

Target JsonTargetProvider::getTarget(int index)
{
  if (index < 0 || index >= this->targetCount) {
    return Target();
  }
  return this->targets[index];
}

JsonTargetProvider::~JsonTargetProvider()
{
  if (this->targets != nullptr) {
    delete[] this->targets;
  }
}