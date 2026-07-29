#include "antidrone_turret/core_controller.hpp"
#include "antidrone_turret/actuator_model.hpp"
#include "antidrone_turret/target_sequence.hpp"
#include <cmath>

constexpr float precision = 1e-5f;

using ActuatorState = antidrone_turret::ActuatorState;
using TargetSample = antidrone_turret::TargetSample;

core::ComputeResult core::CoreController::computeTriggerResult(const TargetSample& target, ActuatorState actuatorState)
{
  ComputeResult result{.action = Action::ACTION_IDLE, .triggerState = TriggerState::TRIGGER_SKIP, .targetState = TargetState::TARGET_NONE};

  if (!target.visible) {
    return result;
  }

  if (target.confidence < confidenceThreshold) {
    result.targetState = TargetState::TARGET_LOW_CONFIDENCE;
    return result;
  }

  // Ціль видима і розпізнавання надійне: наводимось незалежно від дистанції.
  result.action = Action::ACTION_TRACK;
  result.targetState = TargetState::TARGET_LOCKED;
  result.servoCommand = this->computeServoCommand(target.x);
  result.gimbalCommand = this->computeGimbalCommand(target.y);

  // Дистанція вирішує тільки постріл: далі за maxDistance лишається TRIGGER_SKIP.
  if (target.distance_m <= maxDistance) {
    switch (actuatorState) {
      case ActuatorState::kReady:
        result.triggerState = TriggerState::TRIGGER_REQUESTED;
        break;
      case ActuatorState::kReloading:
        result.triggerState = TriggerState::TRIGGER_RELOADING;
        break;
      default:
        break;
    }
  }

  return result;
}

core::ServoCommand core::CoreController::computeServoCommand(float targetX)
{
  ServoCommand command{.targetX = targetX};
  auto errorX = targetX - 320;

  if (std::fabs(errorX) < precision) {
    command.servoDirection = ServoDirection::CENTER;
    command.errorX = 0.0F;
  }
  else if (errorX < 0) {
    command.servoDirection = ServoDirection::LEFT;
    command.errorX = errorX;
  }
  else {
    command.servoDirection = ServoDirection::RIGHT;
    command.errorX = errorX;
  }

  return command;
};

core::GimbalCommand core::CoreController::computeGimbalCommand(float targetY)
{
  GimbalCommand command{.targetY = targetY};

  // y у кадрі росте вниз, тому помилка гімбала інвертована відносно кадру.
  auto errorY = 240 - targetY;

  if (std::fabs(errorY) < precision) {
    command.gimbalDirection = GimbalDirection::CENTER;
    command.errorY = 0.0F;
  }
  else if (errorY > 0) {
    command.gimbalDirection = GimbalDirection::UP;
    command.errorY = errorY;
  }
  else {
    command.gimbalDirection = GimbalDirection::DOWN;
    command.errorY = errorY;
  }

  return command;
};