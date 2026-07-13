#pragma once
#include "interfaces/IBallisticSolver.h"

class AnalyticalSolver : public IBallisticSolver {
private:
  PayloadParams pp;
  DroneConfig droneConfig;
  float payloadTimeOfFlight(float altitude, float speed);
  float calcHDistance(float t, float speed);

public:
  void init(const DroneConfig &droneConfig, const PayloadParams &payloadParams) override;
  BallisticResult solve(float altitude, float speed) override;
};