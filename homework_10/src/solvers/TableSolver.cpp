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
  this->pp = payloadParams;
}

BallisticResult TableSolver::solve()
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

  // Pure ballistics only: the mission processor turns t/h into drop/aim points.
  DEBUG("Ballistic Result: t=" << t << ", h=" << h);

  return BallisticResult{.ok = true, .t = t, .h = h};
}