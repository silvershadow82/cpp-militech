#pragma once

#include "common.hpp"

class IBallisticSolver {
public:
  virtual void init(DroneConfig droneConfig, PayloadParams payloadParams) = 0;
  virtual int solve(SimState& state) = 0;
  virtual ~IBallisticSolver(){};
};

class AnalyticalSolver : public IBallisticSolver {
private:
  PayloadParams pp;
  DroneConfig droneConfig;
  float payloadTimeOfFlight();
  float calcHDistance(float t);

public:
  void init(DroneConfig droneConfig, PayloadParams payloadParams) override;
  int solve(SimState& state) override;
};