#pragma once

#include "Types.h"
#include "interfaces/IBallisticSolver.h"
#include "models/BallisticTable.h"

#include <memory>

class TableSolver : public IBallisticSolver {
private:
  std::unique_ptr<BallisticTable> table;
  std::string tableFilePath;
  PayloadParams pp;

public:
  TableSolver();
  void init(const DroneConfig &droneConfig, const PayloadParams &payloadParams) override;
  BallisticResult solve(float altitude, float speed) override;
};