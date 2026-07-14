#include "providers/JsonTargetProvider.h"
#include "debug.h"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include <fstream>
#include <iostream>
#include <vector>

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
  int targetCount = jt["targetCount"];

  this->targets = std::vector<Target>(targetCount);

  for (int i = 0; i < targetCount; i++) {
    this->targets.emplace_back(Target(i));
    for (int j = 0; j < timeStepCount; j++) {
      this->targets.at(i).setPosAt(j, jt["targets"][i]["positions"][j]["x"], jt["targets"][i]["positions"][j]["y"]);
    }
  }
  targetFile.close();

  LOG("read targetCount=" << targetCount);
  LOG("read timeSteps=" << timeStepCount);

  return 0;
}

JsonTargetProvider::JsonTargetProvider(const std::string &jsonFileName)
  : jsonFileName(jsonFileName)
{
  this->readTargetData();
}

int JsonTargetProvider::getTargetCount()
{
  return this->targets.size();
}

Target JsonTargetProvider::getTarget(int index)
{
  return this->targets.at(index);
}

JsonTargetProvider::~JsonTargetProvider() {}