#pragma once

#include "Types.h"
#include "interfaces/IDroneState.h"
#include "models/Target.h"
#include <memory>
#include <vector>

class IBallisticSolver;
class StatCollector;
class ITargetProvider;
class IConfigLoader;

class MissionProcessor {
private:
  bool initialized{false};
  bool done{false};
  MissionContext context;
  std::vector<float> timeToTargets;
  std::unique_ptr<IDroneState> currentState;
  std::unique_ptr<StatCollector> statCollector;
  std::unique_ptr<IBallisticSolver> solver;
  std::unique_ptr<ITargetProvider> targetProvider;
  std::unique_ptr<IConfigLoader> configLoader;
  std::string dataFolder;

  void initContext(const DroneConfig &config, const int targetCount);
  bool computeFirePoint(const Target &target);
  float leadTimeToTarget(const Target &target);
  void updateDroneState();

public:
  MissionProcessor(std::unique_ptr<IBallisticSolver> solver,
                   std::unique_ptr<ITargetProvider> targetProvider,
                   std::unique_ptr<IConfigLoader> configLoader);
  ~MissionProcessor();
  bool init(ConfigSource configSource, const std::string &dataFolder);
  bool hasNext();
  bool step();
  void reset();
  void changeSolver(std::unique_ptr<IBallisticSolver> solver);

  int totalSteps();
  void printStats();
};