#pragma once
#include <memory>
#include "interfaces/IBallisticSolver.h"
#include "models/BallisticTable.h"

class TableSolver : public IBallisticSolver {
private:
  std::unique_ptr<BallisticTable> table;

public:
  void init(const DroneConfig &droneConfig, const PayloadParams &payloadParams) override;
  BallisticResult solve(Coord dronePos, Coord targetPos, float droneAngle) override;
};