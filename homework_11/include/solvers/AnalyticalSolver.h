#pragma once
#include "interfaces/IBallisticSolver.h"

class AnalyticalSolver : public IBallisticSolver {
private:
  PayloadParams pp;
  DroneConfig droneConfig;
  float payloadTimeOfFlight();
  float calcHDistance(float t);

public:
  void init(const DroneConfig &droneConfig, const PayloadParams &payloadParams) override;
  BallisticResult solve() override;
};