#pragma once

#include "Types.h"
#include "interfaces/IBallisticSolver.h"
#include "models/Coord.h"
#include "models/Target.h"
#include "models/Telem.h"

#include <memory>

struct FireSolution
{
  bool ok{false};
  Coord dropPoint{};
  float aimAngle{0.f};
  float desiredSpeed{0.f};
  float dropTime{0.f};
  bool inDropWindow{false};
};

class FireGeometry
{
private:
  DroneConfig config;
  std::unique_ptr<IBallisticSolver> solver;

public:
  FireGeometry(const DroneConfig &config, std::unique_ptr<IBallisticSolver> solver);
  FireSolution solve(const Telem &telem, const Target &target);
};
