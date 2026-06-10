#pragma once

#include "common.hpp"
// ------------------------- Utility methods -----------------------
int minValueIdx(float array[], size_t len);
float timeToDistance(float distance, float currentSpeed, float attackSpeed, float acceleration, float accPath);
void normalizeAngle(float &angleDiff);
float convergeAngle(float &droneAngle, float targetAngle, const DroneConfig &droneConfig);