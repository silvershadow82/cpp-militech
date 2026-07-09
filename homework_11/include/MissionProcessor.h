#pragma once

#include "Types.h"
#include "DronePhysics.h"
#include "interfaces/IDroneState.h"
#include "models/Target.h"
#include <atomic>
#include <memory>
#include <vector>

class IBallisticSolver;
class StatCollector;
class ITargetProvider;
class IConfigLoader;
class DronePhysics;

class MissionProcessor {
private:
  bool initialized{false};
  bool done{false};
  std::atomic<bool> stopFlag{false};
  MissionContext context;
  std::vector<float> timeToTargets;
  std::unique_ptr<IDroneState> currentState;
  std::unique_ptr<StatCollector> statCollector;
  std::unique_ptr<IBallisticSolver> solver;
  std::unique_ptr<ITargetProvider> targetProvider;
  std::unique_ptr<IConfigLoader> configLoader;
  std::unique_ptr<DronePhysics> dronePhysics;
  std::string dataFolder;

  void initContext(const DroneConfig &config, const int targetCount);
  bool computeFireGeometry(const Coord &dronePos, const Coord &targetPos, float droneAngle, const BallisticResult &ballistics);
  bool computeFirePoint(const Target &target);
  float leadTimeToTarget(const Target &target);
  void updateDroneState();

public:
  MissionProcessor(std::unique_ptr<IBallisticSolver> solver,
                   std::unique_ptr<ITargetProvider> targetProvider,
                   std::unique_ptr<IConfigLoader> configLoader,
                   std::unique_ptr<DronePhysics> dronePhysics);
  ~MissionProcessor();
  bool init(ConfigSource configSource, const std::string &dataFolder);
  bool hasNext();
  bool step();
  void run();
  void stop();
  void reset();
  void changeSolver(std::unique_ptr<IBallisticSolver> solver);

  int totalSteps();
  void printStats();
};