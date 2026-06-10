#include "util.hpp"

int minValueIdx(float array[], size_t len)
{
  int idx{0};
  float value{array[idx]};
  for (int i = 1; i < len; i++) {
    if (value > array[i]) {
      value = array[i];
      idx = i;
    }
  }
  return idx;
}

float timeToDistance(float distance, float currentSpeed, float attackSpeed, float acceleration, float accPath)
{
  float accTime = (attackSpeed - currentSpeed) / acceleration;
  float diff = std::max(distance - accPath, 0.f);
  return static_cast<float>(accTime + diff / attackSpeed);
}

void normalizeAngle(float &angleDiff)
{
  while (angleDiff > M_PI)
    angleDiff -= static_cast<float>(2.f * M_PI);
  while (angleDiff < -M_PI)
    angleDiff += static_cast<float>(2.f * M_PI);
}

float convergeAngle(float &droneAngle, float targetAngle, const DroneConfig &droneConfig)
{
  float angleDiff = targetAngle - droneAngle;

  normalizeAngle(angleDiff);

  float maxAngleStep = droneConfig.angularSpeed * droneConfig.simTimeStep;

  if (fabs(angleDiff) <= maxAngleStep) {
    droneAngle = targetAngle;
  }
  else if (angleDiff > 0) {
    droneAngle += maxAngleStep;
  }
  else {
    droneAngle -= maxAngleStep;
  }

  normalizeAngle(droneAngle);

  return fabs(angleDiff);
}