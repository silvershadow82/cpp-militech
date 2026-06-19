#pragma once
#include "Types.h"
#include <string>
#include <vector>

constexpr int MAX_STEPS{10000};

class StatCollector {
private:
  std::string outputFileName;
  std::vector<SimStep> statSteps;

public:
  StatCollector(const std::string &outputFileName);
  void collectStateStepStats(const SimState &state);
  void resetStats();
  void printStats();
};