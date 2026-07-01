#pragma once

#include "interfaces/ITargetProvider.h"
#include "models/Coord.h"
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

class ThreadSafeTargetProvider : public ITargetProvider {
private:
  int arrayTimeStep;
  std::mutex mtx;
  std::atomic<bool> stopFlag{false};
  std::atomic<int> currentTimeStep{0};
  std::string jsonFileName;
  std::vector<Target> targets;
  std::vector<std::vector<Coord>> timeSteps;

  int readTargetData();
  Coord targetSpeed(int targetId, int timeStep);

public:
  ThreadSafeTargetProvider(const std::string &jsonFileName, int arrayTimeStep);
  ~ThreadSafeTargetProvider();

  bool isThreadReady();
  void start();
  void stop();
  void run();
  int getTargetCount() override;
  Target getTarget(int) override;
};