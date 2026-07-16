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

void StatCollector::collectStateStepStats(const MissionContext &ctx)
{
  this->statSteps[ctx.step].pos = ctx.dronePos;
  this->statSteps[ctx.step].direction = ctx.droneAngle;
  this->statSteps[ctx.step].targetIdx = ctx.lastTargetIdx;
  this->statSteps[ctx.step].dropPoint = ctx.dropPoint;
  this->statSteps[ctx.step].state = ctx.state;
  this->statSteps[ctx.step].predictedTarget = ctx.predictedTargetPos;
  this->statSteps[ctx.step].aimPoint = ctx.aimPoint;
  this->statSteps[ctx.step].timeSecSinceStart = ctx.timeSecSinceStart;
  this->filledSteps++;
}

void StatCollector::printStats()
{
  std::ofstream output(this->outputFileName);

  if (!output.is_open()) {
    std::cerr << "Unable to open simulation.json" << '\n';
    return;
  }

  json out{};
  out["totalSteps"] = this->filledSteps;
  out["steps"] = json::array();

  for (int i = 0; i < filledSteps; i++) {
    auto statStep = this->statSteps[i];
    json step{};
    step["position"] = {{"x", statStep.pos.x}, {"y", statStep.pos.y}};
    step["direction"] = statStep.direction;
    step["state"] = (int)statStep.state;
    step["targetIndex"] = statStep.targetIdx;
    step["dropPoint"] = {{"x", statStep.dropPoint.x}, {"y", statStep.dropPoint.y}};
    step["aimPoint"] = {{"x", statStep.aimPoint.x}, {"y", statStep.aimPoint.y}};
    step["predictedTarget"] = {{"x", statStep.predictedTarget.x}, {"y", statStep.predictedTarget.y}};
    step["timeSecSinceStart"] = statStep.timeSecSinceStart;

    out["steps"].push_back(step);
  }

  output << out.dump(2);

  output.close();
}

void StatCollector::resetStats()
{
  this->statSteps.clear();
}