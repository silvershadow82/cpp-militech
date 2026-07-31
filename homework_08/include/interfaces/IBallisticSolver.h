#pragma once

#include "Types.h"

class IBallisticSolver {
public:
  virtual void init(const DroneConfig &droneConfig, const PayloadParams &payloadParams) = 0;
  virtual BallisticResult solve(Coord dronePos, Coord targetPos, float droneAngle) = 0;
  virtual ~IBallisticSolver() {};
};
