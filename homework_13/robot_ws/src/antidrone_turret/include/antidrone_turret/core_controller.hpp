#pragma once

#include <cstdint>
#include "antidrone_turret/actuator_model.hpp"

namespace core {

enum class Action : uint8_t { IDLE, TRACK };
enum class TriggerState : uint8_t { REQUESTED, RELOADING, SKIP };
enum class ServoDirection : int { LEFT = -1, CENTER, RIGHT };
enum class GimbalDirection : int { DOWN = -1, CENTER, UP };

struct TriggerResult {
  TriggerState triggerState;
  Action action;
  ServoDirection servoDirection;
  GimbalDirection gimbalDirection;
};

class CoreController {
private:
  float confidenceThreshold;
  float maxDistance;

  ServoDirection computeServoDirection(float targetX);
  GimbalDirection computeGimbalDirection(float targetX);

public:
  CoreController(float confidenceThreshold, float maxDistance)
    : confidenceThreshold(confidenceThreshold)
    , maxDistance(maxDistance)
  {
  }

  TriggerResult computeTriggerResult(
    bool targetVisible, float confidence, float targetX, float targetY, float distance, antidrone_turret::ActuatorState actuatorState);
};

}  // namespace core