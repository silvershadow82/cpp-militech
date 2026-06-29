#pragma once
#include "Types.h"
#include <string>
#include <vector>

constexpr int MAX_STEPS{10000};

class StatCollector {
private:
  std::string outputFileName;
  std::vector<SimStep> statSteps;
  int filledSteps{0};

public:
  StatCollector(const std::string &outputFileName);
  void collectStateStepStats(const MissionContext &ctx);
  void resetStats();
  void printStats();
};