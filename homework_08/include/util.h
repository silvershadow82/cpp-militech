#pragma once

#include "Types.h"
#include <vector>

namespace util {
// ------------------------- Utility methods -----------------------
int minValueIdx(const std::vector<float> &array);
float timeToDistance(float distance, float currentSpeed, float attackSpeed, float acceleration, float accPath);
void normalizeAngle(float &angleDiff);
float convergeAngle(float &droneAngle, float targetAngle, const DroneConfig &droneConfig);
}  // namespace util
