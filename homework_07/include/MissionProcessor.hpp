#pragma once

#include "common.hpp"
#include "ComponentFactory.hpp"

class MissionProcessor {
private:
  bool initialized{false};
  bool done{false};
  SimState state;
  SimStep* statSteps;
  float* timeToTargets;
  IBallisticSolver* solver;
  ITargetProvider* targetProvider;
  IConfigLoader* configLoader;
  std::string dataFolder;

  void initState(const DroneConfig& config, const int targetCount);
  bool computeFirePoint(const Target& target);
  float leadTimeToTarget(const Target& target);
  void updateDroneState();
  void collectCurrentStepStats();

public:
  bool init(ConfigSource configSource, const std::string& dataFolder);
  bool hasNext();
  bool step();
  void reset();
  void changeSolver(IBallisticSolver* solver);

  int totalSteps();
  void printStats();
  ~MissionProcessor();
};