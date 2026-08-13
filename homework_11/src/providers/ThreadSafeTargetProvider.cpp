#include "providers/ThreadSafeTargetProvider.h"
#include "models/Coord.h"
#include "models/Target.h"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include "debug.h"
#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>
#include <vector>

using json = nlohmann::json;

ThreadSafeTargetProvider::ThreadSafeTargetProvider(const std::string &jsonFileName, float arrayTimeStep, float timeScale)
  : jsonFileName(jsonFileName)
  , arrayTimeStep(arrayTimeStep)
  , timeScale(timeScale)
{
  this->readTargetData();

  // Ініціалізуємо поточний крок часу та встановлюємо початкові позиції та швидкості цілей
  this->currentTimeStep.store(0);
  for (int i = 0; i < this->targets.size(); i++) {
    if (i < this->timeSteps.size() && !this->timeSteps[i].empty()) {
      this->targets[i].pos = this->timeSteps[i][0];
      this->targets[i].velocity = this->targetSpeed(i, 0);
    }
  }
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
    std::vector<Coord> steps = std::vector<Coord>();
    steps.reserve(timeStepCount);
    this->targets.emplace_back(Target{.index = i, .pos = Coord(), .velocity = Coord()});
    this->timeSteps.emplace_back(steps);
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
  int length = this->timeSteps.empty() ? 0 : (int)this->timeSteps[targetId].size();

  if (timeStep < 0 || timeStep >= length) {
    return Coord(0.0f, 0.0f);  // Invalid time step
  }

  // Зациклюємо індекс попереднього кроку часу
  int prevTimeStep = (timeStep - 1 + length) % length;

  const Coord &prevPos = this->timeSteps[targetId][prevTimeStep];
  const Coord &currPos = this->timeSteps[targetId][timeStep];

  float dx = currPos.x - prevPos.x;
  float dy = currPos.y - prevPos.y;

  return Coord(dx / this->arrayTimeStep, dy / this->arrayTimeStep);
}

bool ThreadSafeTargetProvider::isThreadReady()
{
  return this->ready.load();
}

void ThreadSafeTargetProvider::start()
{
  this->started.store(true);
}

void ThreadSafeTargetProvider::stop()
{
  this->stopFlag.store(true);
}

void ThreadSafeTargetProvider::run()
{
  LOG("ThreadSafeTargetProvider thread up (run() entry)");

  this->ready.store(true);

  while (!this->started.load() && !this->stopFlag.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  int length = this->timeSteps.empty() ? 0 : this->timeSteps[0].size();

  while (!this->stopFlag.load()) {
    {
      std::lock_guard<std::mutex> lock(mtx);
      int step = this->currentTimeStep.load();
      for (int i = 0; i < this->targets.size(); i++) {
        this->targets[i].pos = this->timeSteps[i][step];
        this->targets[i].velocity = this->targetSpeed(i, step);
      }
    }
    // Зациклюємо індекс поточного кроку часу, якщо є дані для оновлення
    if (length > 0) {
      this->currentTimeStep.store((this->currentTimeStep.load() + 1) % length);
    }
    // Спимо на час, що відповідає кроку часу масиву, з урахуванням масштабу часу
    std::this_thread::sleep_for(std::chrono::duration<float>(this->arrayTimeStep / this->timeScale));
  }
}

int ThreadSafeTargetProvider::getTargetCount() const
{
  {
    std::lock_guard<std::mutex> lock(mtx);
    return targets.size();
  }
}

Target ThreadSafeTargetProvider::getTarget(int index) const
{
  {
    std::lock_guard<std::mutex> lock(mtx);
    if (index >= 0 && index < static_cast<int>(targets.size())) {
      return targets[index];
    }
  }
  // Якщо індекс недійсний, повертаємо порожню ціль
  LOG("Warning: getTarget() called with invalid index " << index);
  return Target();
}