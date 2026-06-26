#include "debug.h"
#include "solvers/TableSolver.h"
#include "Types.h"
#include "models/BallisticTable.h"
#include "models/Coord.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#ifndef BALLISTIC_TABLE_PATH
#define BALLISTIC_TABLE_PATH "table/ballistic_table.txt"
#endif

TableSolver::TableSolver()
  : tableFilePath(BALLISTIC_TABLE_PATH)
{
}

void TableSolver::init(const DroneConfig& config, const PayloadParams& payloadParams)
{
  this->table = std::make_unique<BallisticTable>();
  bool loaded = this->table->load(this->tableFilePath.c_str());
  if (!loaded) {
    throw std::runtime_error("Unable to load table file: " + this->tableFilePath);
  }

  LOG("Ballistic table loaded successfully");

  this->altitude = config.altitude;
  this->speed = config.attackSpeed;
  this->accelPath = config.accelPath;
  this->pp = payloadParams;
}

BallisticResult TableSolver::solve(Coord dronePos, Coord targetPos, float droneAngle)
{
  auto result = this->table->lookup(this->altitude, this->speed, this->pp.m, this->pp.d, this->pp.l);

  float t = result.t;
  float h = result.hDist;

  if (t <= 0) {
    std::cout << "Invalid t=" << t << std::endl;
    return BallisticResult{.ok = false};
  }

  if (h <= 0) {
    std::cout << "Invalid h=" << h << std::endl;
    return BallisticResult{.ok = false};
  }

  // Calculate drone to target distance
  float distanceToTarget = Coord::distance(dronePos, targetPos);

  if (distanceToTarget <= 0) {
    std::cout << "Invalid D=" << distanceToTarget << std::endl;
    return BallisticResult{.ok = false};
  }
  // Check if drone has to maneuvre and calculate new xd, yd
  Coord newDronePos = dronePos;

  if (h + this->accelPath > distanceToTarget) {
    if (fabsf(distanceToTarget) < 1e-6) {
      newDronePos.x = targetPos.x - (h + this->accelPath);
      newDronePos.y = targetPos.y;

      distanceToTarget = h + this->accelPath;
    }
    else {
      newDronePos = targetPos - (targetPos - dronePos) * (h + this->accelPath) / distanceToTarget;
      distanceToTarget = Coord::distance(newDronePos, targetPos);
    }
  }

  float ratio = (distanceToTarget - h) / distanceToTarget;

  // Calculate drop point coordinates
  Coord dir = {static_cast<float>(cos(droneAngle)), static_cast<float>(sin(droneAngle))};

  BallisticResult ret{
    .ok = true, .dropPoint = newDronePos + (targetPos - newDronePos) * ratio, .aimPoint = newDronePos + dir * h, .payloadDropTime = t};

  DEBUG("Ballistic Result: dropPoint=(" << ret.dropPoint.x << "," << ret.dropPoint.y << ")");

  return ret;
}