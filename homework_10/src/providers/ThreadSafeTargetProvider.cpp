#include "providers/ThreadSafeTargetProvider.h"
#include "models/Coord.h"
#include "models/Target.h"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include "debug.h"
#include <thread>
#include <fstream>
#include <iostream>
#include <vector>

using json = nlohmann::json;

ThreadSafeTargetProvider::ThreadSafeTargetProvider(const std::string &jsonFileName, int arrayTimeStep)
  : jsonFileName(jsonFileName)
  , arrayTimeStep(arrayTimeStep)
{
  this->readTargetData();
}

ThreadSafeTargetProvider::~ThreadSafeTargetProvider() {}

int ThreadSafeTargetProvider::readTargetData()
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

  this->targets.clear();
  this->targets.reserve(targetCount);

  for (int i = 0; i < targetCount; i++) {
    this->targets.emplace_back(Target{.pos = Coord(), .velocity = Coord()});
    this->timeSteps.emplace_back(std::vector<Coord>());
    for (int j = 0; j < timeStepCount; j++) {
      this->timeSteps[i].emplace_back(Coord(jt["targets"][i]["positions"][j]["x"], jt["targets"][i]["positions"][j]["y"]));
    }
  }
  targetFile.close();

  LOG("read targetCount=" << targetCount);
  LOG("read timeSteps=" << timeStepCount);

  return 0;
}

Coord ThreadSafeTargetProvider::targetSpeed(int targetId, int timeStep)
{
  if (timeStep < 0 || timeStep > this->timeSteps.size()) {
    return Coord(0.0f, 0.0f);  // Invalid time step
  }

  // wrap timeStep to ensure the previous of 0 is the last time step
  float prevTimeStep = (timeStep - 1 + this->timeSteps.size()) % this->timeSteps.size();

  const Coord &prevPos = this->timeSteps[prevTimeStep][targetId];
  const Coord &currPos = this->timeSteps[timeStep][targetId];

  float dx = currPos.x - prevPos.x;
  float dy = currPos.y - prevPos.y;

  return Coord(dx / this->arrayTimeStep, dy / this->arrayTimeStep);
}

bool ThreadSafeTargetProvider::isThreadReady()
{
  return !this->stopFlag.load();
}

void ThreadSafeTargetProvider::start()
{
  this->stopFlag.store(false);
}

void ThreadSafeTargetProvider::stop()
{
  this->stopFlag.store(true);
}

void ThreadSafeTargetProvider::run()
{
  if (this->stopFlag.load()) {
    return;
  }
  {
    // Lock the mutex while updating targets to ensure thread safety
    std::lock_guard<std::mutex> lock(mtx);
    for (int i = 0; i < this->targets.size(); i++) {
      int step = this->currentTimeStep.load();
      this->targets[i].pos = this->timeSteps[step][i];
      this->targets[i].velocity = this->targetSpeed(i, step);
    }
  }
  // Increment the current time step and wrap around if necessary
  this->currentTimeStep.store((this->currentTimeStep.load() + 1) % this->timeSteps.size());
  // Sleep for the specified arrayTimeStep duration before the next update
  std::this_thread::sleep_for(std::chrono::seconds(this->arrayTimeStep));
}

int ThreadSafeTargetProvider::getTargetCount()
{
  {
    std::lock_guard<std::mutex> lock(mtx);
    return targets.size();
  }
}

Target ThreadSafeTargetProvider::getTarget(int index)
{
  {
    std::lock_guard<std::mutex> lock(mtx);
    if (index >= 0 && index < targets.size()) {
      return targets[index];
    }
  }
  // Handle out-of-bounds access, possibly throw an exception or return a default Target
  return Target();  // Placeholder return value
}