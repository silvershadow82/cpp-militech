#include "antidrone_turret/core_controller.hpp"
#include "antidrone_turret/actuator_model.hpp"
#include <cmath>

constexpr float precision = 1e-5f;

core::TriggerResult core::CoreController::computeTriggerResult(
  bool targetVisible, float confidence, float targetX, float targetY, float distance, antidrone_turret::ActuatorState actuatorState)
{
  TriggerResult result{.triggerState = TriggerState::SKIP, .action = Action::IDLE};

  if (!targetVisible || confidence < confidenceThreshold || distance > maxDistance) {
    return result;
  }

  if (distance <= maxDistance) {
    result.action = Action::TRACK;
    result.servoDirection = this->computeServoDirection(targetX);
    result.gimbalDirection = this->computeGimbalDirection(targetY);
    switch (actuatorState) {
      case antidrone_turret::ActuatorState::kReady:
        result.triggerState = TriggerState::REQUESTED;
        break;
      case antidrone_turret::ActuatorState::kReloading:
        result.triggerState = TriggerState::RELOADING;
        break;
    }
  }

  return result;
}

core::ServoDirection core::CoreController::computeServoDirection(float targetX)
{
  auto diff = targetX - 320;

  if (std::fabs(diff) < precision) {
    return ServoDirection::CENTER;
  }
  return (diff < 0) ? ServoDirection::LEFT : ServoDirection::RIGHT;
};

core::GimbalDirection core::CoreController::computeGimbalDirection(float targetY)
{
  auto diff = targetY - 240;

  if (std::fabs(diff) < precision) {
    return GimbalDirection::CENTER;
  }
  return (diff < 0) ? GimbalDirection::UP : GimbalDirection::DOWN;
};