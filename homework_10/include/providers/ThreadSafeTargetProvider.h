#pragma once

#include "interfaces/ITargetProvider.h"
#include "models/Coord.h"
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

class ThreadSafeTargetProvider : public ITargetProvider {
private:
  float arrayTimeStep;
  float timeScale;
  mutable std::mutex mtx;
  std::atomic<bool> stopFlag{false};
  std::atomic<bool> ready{false};
  std::atomic<bool> started{false};
  std::atomic<int> currentTimeStep{0};
  std::string jsonFileName;
  std::vector<Target> targets;
  std::vector<std::vector<Coord>> timeSteps;

  int readTargetData();
  Coord targetSpeed(int targetId, int timeStep);

public:
  ThreadSafeTargetProvider(const std::string &jsonFileName, float arrayTimeStep, float timeScale);
  ~ThreadSafeTargetProvider();

  bool isThreadReady();
  void start();
  void stop();
  void run();
  int getTargetCount() const override;
  Target getTarget(int) const override;
};