#include "StatCollector.h"
#include "Types.h"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include <fstream>
#include <iostream>
#include <vector>

using json = nlohmann::json;

StatCollector::StatCollector(const std::string &outputFileName)
  : outputFileName(outputFileName)
{
  this->statSteps = std::vector<SimStep>(MAX_STEPS);
};

void StatCollector::collectStateStepStats(const DroneContext &ctx)
{
  statSteps[ctx.step].pos = ctx.dronePos;
  statSteps[ctx.step].direction = ctx.droneAngle;
  statSteps[ctx.step].targetIdx = ctx.lastTargetIdx;
  statSteps[ctx.step].dropPoint = ctx.dropPoint;
  statSteps[ctx.step].state = ctx.state;
  statSteps[ctx.step].predictedTarget = ctx.predictedTargetPos;
  statSteps[ctx.step].aimPoint = ctx.aimPoint;
}

void StatCollector::printStats()
{
  std::ofstream output(this->outputFileName);

  if (!output.is_open()) {
    std::cerr << "Unable to open simulation.json" << '\n';
    return;
  }

  json out{};
  const int stepCount = this->statSteps.size();

  out["totalSteps"] = stepCount;
  out["steps"] = json::array();

  for (auto statStep : this->statSteps) {
    json step{};
    step["position"] = {{"x", statStep.pos.x}, {"y", statStep.pos.y}};
    step["direction"] = statStep.direction;
    step["state"] = statStep.state;
    step["targetIndex"] = statStep.targetIdx;
    step["dropPoint"] = {{"x", statStep.dropPoint.x}, {"y", statStep.dropPoint.y}};
    step["aimPoint"] = {{"x", statStep.aimPoint.x}, {"y", statStep.aimPoint.y}};
    step["predictedTarget"] = {{"x", statStep.predictedTarget.x}, {"y", statStep.predictedTarget.y}};

    out["steps"].push_back(step);
  }

  output << out.dump(2);

  output.close();
}

void StatCollector::resetStats()
{
  this->statSteps.clear();
}