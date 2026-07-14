#include "util.h"
#include <cmath>

int util::minValueIdx(const std::vector<float> &array)
{
  int idx{0};
  float value{array[idx]};
  for (int i = 1; i < array.size(); i++) {
    if (array[i] < value) {
      value = array[i];
      idx = i;
    }
  }
  return idx;
}

float util::timeToDistance(float distance, float currentSpeed, float attackSpeed, float acceleration, float accPath)
{
  float accTime = (attackSpeed - currentSpeed) / acceleration;
  float diff = std::max(distance - accPath, 0.f);
  return static_cast<float>(accTime + diff / attackSpeed);
}

void util::normalizeAngle(float &angleDiff)
{
  while (angleDiff > M_PI)
    angleDiff -= static_cast<float>(2.f * M_PI);
  while (angleDiff < -M_PI)
    angleDiff += static_cast<float>(2.f * M_PI);
}

float util::convergeAngle(float &droneAngle, float targetAngle, const DroneConfig &droneConfig)
{
  float angleDiff = targetAngle - droneAngle;

  util::normalizeAngle(angleDiff);

  float maxAngleStep = droneConfig.angularSpeed * droneConfig.simTimeStep;

  if (fabsf(angleDiff) <= maxAngleStep) {
    droneAngle = targetAngle;
  }
  else if (angleDiff > 0) {
    droneAngle += maxAngleStep;
  }
  else {
    droneAngle -= maxAngleStep;
  }

  util::normalizeAngle(droneAngle);

  return fabsf(angleDiff);
}
