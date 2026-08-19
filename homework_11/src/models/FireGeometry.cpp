#include "models/FireGeometry.h"

#include <cmath>
#include <utility>

// Максимальне відхилення від точки скиду, можна коригувати для кращих результатів
constexpr float dropDistanceThreshold{1.0f};

FireGeometry::FireGeometry(const DroneConfig &config, std::unique_ptr<IBallisticSolver> solver)
  : config(config)
  , solver(std::move(solver))
{
}

FireSolution FireGeometry::solve(const Telem &telem, const Target &target)
{
  FireSolution solution{};

  BallisticResult ballistics = this->solver->solve(telem.altitude, telem.speed);

  if (!ballistics.ok) {
    return solution;
  }

  const float t = ballistics.t;
  const float h = ballistics.h;

  const Coord dronePos = telem.pos;
  const Coord leadPos = target.pos + target.velocity * t;

  float distanceToTarget = Coord::distance(dronePos, leadPos);

  // Дрон прямо над ціллю
  if (distanceToTarget <= 0) {
    return solution;
  }

  // Обчислення проміжної точки
  Coord newDronePos = dronePos;
  if (h + this->config.accelPath > distanceToTarget) {
    if (fabsf(distanceToTarget) < 1e-6f) {
      newDronePos.x = leadPos.x - (h + this->config.accelPath);
      newDronePos.y = leadPos.y;
      distanceToTarget = h + this->config.accelPath;
    }
    else {
      newDronePos = leadPos - (leadPos - dronePos) * (h + this->config.accelPath) / distanceToTarget;
      distanceToTarget = Coord::distance(newDronePos, leadPos);
    }
  }

  const float ratio = (distanceToTarget - h) / distanceToTarget;

  solution.dropPoint = newDronePos + (leadPos - newDronePos) * ratio;
  solution.aimAngle = Coord::angle(dronePos, leadPos);
  solution.desiredSpeed = this->config.attackSpeed;
  solution.dropTime = t;

  // Обчислення точки скиду, чи попадаємо у вікно
  const Coord dir{static_cast<float>(cos(telem.angle)), static_cast<float>(sin(telem.angle))};
  const Coord predictedHit = dronePos + dir * h;
  const float deviation = Coord::distance(predictedHit, leadPos);
  const bool nearDropPoint = Coord::distance(dronePos, solution.dropPoint) <= dropDistanceThreshold;

  solution.inDropWindow = nearDropPoint && (deviation <= this->config.hitRadius);
  solution.ok = true;

  return solution;
}
