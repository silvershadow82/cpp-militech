#pragma once

#include "Types.h"
#include "models/Target.h"
#include <vector>

class IBallisticSolver;
class StatCollector;
class ITargetProvider;
class IConfigLoader;

class MissionProcessor {
private:
  bool initialized{false};
  bool done{false};
  SimState state;
  std::vector<float> timeToTargets;
  StatCollector *statCollector;
  IBallisticSolver *solver;
  ITargetProvider *targetProvider;
  IConfigLoader *configLoader;
  std::string dataFolder;

  void initState(const DroneConfig &config, const int targetCount);
  bool computeFirePoint(const Target &target);
  float leadTimeToTarget(const Target &target);
  void updateDroneState();

public:
  MissionProcessor(IBallisticSolver *solver, ITargetProvider *targetProvider, IConfigLoader *configLoader);
  bool init(ConfigSource configSource, const std::string &dataFolder);
  bool hasNext();
  bool step();
  void reset();
  void changeSolver(IBallisticSolver *solver);

  int totalSteps();
  void printStats();
  ~MissionProcessor();
};