#include "IBallisticSolver.hpp"
#include <iostream>

float AnalyticalSolver::payloadTimeOfFlight()
{
  // Calculate the payload travel time t
  // Using Cardano formula to the cubic equation at^3+bt^2+c = 0
  float altitude = this->droneConfig.altitude;
  float speed = this->droneConfig.attackSpeed;

  float a = static_cast<float>(pp.d * G * pp.m - 2.0 * pp.d2 * pp.l * speed);
  float b = static_cast<float>(-3.0 * G * pp.m2 + 3.0 * pp.d * pp.l * pp.m * speed);
  float c = static_cast<float>(6.0 * pp.m2 * altitude);
  float a2 = a * a, a3 = a2 * a;
  float b2 = b * b, b3 = b2 * b;

  // Cardano trigonometric solution
  float p = static_cast<float>(-b2 / (3.0 * a2));
  float q = static_cast<float>((2.0 * b3) / (27.0 * a3) + c / a);
  float arg = static_cast<float>(((3.0 * q) / (2.0 * p)) * sqrt(-3.0 / p));

  // Check that arg is within the range [-1, 1]
  if (arg < -1 || arg > 1) {
    std::cerr << "Warning: acos arg is out of range: " << arg << std::endl;
    return -1.f;
  }

  float phi = acos(arg);

  // Calculate payload travel time
  float t = static_cast<float>(2.0 * sqrt(-p / 3.0) * cos((phi + 4.0 * M_PI) / 3.0) - b / (3.0 * a));

  return t;
}

float AnalyticalSolver::calcHDistance(float t)
{
  float t2 = t * t;
  float t3 = t2 * t;
  float t4 = t2 * t2;
  float t5 = t3 * t2;

  float speed = this->droneConfig.attackSpeed;
  float e1 = t * speed;
  float e2 = static_cast<float>(-t2 * pp.d * speed / (2.0 * pp.m));
  float e3 = static_cast<float>(t3 * (6.0 * pp.d * G * pp.l * pp.m - 6.0 * pp.d2 * (pp.l2 - 1.0) * speed) / (36.0 * pp.m2));
  float e4 = static_cast<float>(t4 *
                                (3.0 * pp.d3 * (pp.l2 + 1.0) * pp.l2 * speed + 6.0 * pp.d3 * (pp.l2 + 1.0) * pp.l4 * speed -
                                 6.0 * pp.d2 * G * (pp.l4 + pp.l2 + 1.0) * pp.l * pp.m) /
                                (36.0 * pow((pp.l2 + 1.0), 2) * pp.m3));
  float e5 = static_cast<float>(t5 * (3.0 * pp.d3 * G * pp.l3 * pp.m - 3.0 * pp.d4 * pp.l2 * (pp.l2 + 1.0) * speed) /
                                (36.0 * (pp.l2 + 1.0) * pp.m4));

  // Calculate the payload travel distance h
  float h = e1 + e2 + e3 + e4 + e5;

  return h;
}

void AnalyticalSolver::init(DroneConfig droneConfig, PayloadParams payloadParams)
{
  this->droneConfig = droneConfig;
  this->pp = payloadParams;
}
int AnalyticalSolver::solve(SimState& state)
{
  float t = payloadTimeOfFlight();

  if (t <= 0) {
    std::cout << "Invalid t=" << t << std::endl;
    return 1;
  }

  float h = calcHDistance(t);

  if (h <= 0) {
    std::cout << "Invalid h=" << h << std::endl;
    return 1;
  }

  // Calculate drone to target distance
  float distanceToTarget = Coord::distance(state.dronePos, state.targetPos);

  if (distanceToTarget <= 0) {
    std::cout << "Invalid D=" << distanceToTarget << std::endl;
    return 1;
  }
  // Check if drone has to maneuvre and calculate new xd, yd

  if (h + droneConfig.accelPath > distanceToTarget) {
    if (fabs(distanceToTarget) < 1e-6) {
      state.dronePos.x = state.targetPos.x - (h + droneConfig.accelPath);
      state.dronePos.y = state.targetPos.y;
      // outDronePos.x = targetPos.x - (h + accelerationPath);

      // DEBUG("with intermediate point: NewDronePos=(" << state.dronePos.x << "," << state.dronePos.y << ")");

      distanceToTarget = h + droneConfig.accelPath;
    }
    else {
      state.dronePos = state.targetPos - (state.targetPos - state.dronePos) * (h + droneConfig.accelPath) / distanceToTarget;

      // DEBUG("NewDronePos=(" << state.dronePos.x << ", " << state.dronePos.y << ")");
      // xd = targetX - (targetX - xd) * (h + accelerationPath) / distanceToTarget;
      // yd = targetY - (targetY - yd) * (h + accelerationPath) / distanceToTarget;
      distanceToTarget = Coord::distance(state.dronePos, state.targetPos);
    }
  }

  float ratio = (distanceToTarget - h) / distanceToTarget;

  // Calculate drop point coordinates
  Coord dir = {static_cast<float>(cos(state.droneAngle)), static_cast<float>(sin(state.droneAngle))};
  state.dropPoint = state.dronePos + (state.targetPos - state.dronePos) * ratio;
  state.aimPoint = state.dronePos + dir * h;
  state.payloadDropTime = t;

  return 0;
}