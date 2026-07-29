#pragma once

#include <cstdint>

#include "antidrone_turret/actuator_model.hpp"
#include "antidrone_turret/target_sequence.hpp"

namespace core {

constexpr float frameCenterX = 320.0F;
constexpr float frameCenterY = 240.0F;

enum class Action : uint8_t { ACTION_IDLE, ACTION_TRACK };
enum class TriggerState : uint8_t { TRIGGER_SKIP, TRIGGER_REQUESTED, TRIGGER_RELOADING };
enum class ServoDirection : int { LEFT = -1, CENTER, RIGHT };
enum class GimbalDirection : int { DOWN = -1, CENTER, UP };
enum class TargetState : uint8_t { TARGET_NONE, TARGET_LOW_CONFIDENCE, TARGET_LOCKED };

struct ServoCommand {
  ServoDirection servoDirection{ServoDirection::CENTER};
  float targetX{0.0F};
  float errorX{0.0F};
};

struct GimbalCommand {
  GimbalDirection gimbalDirection{GimbalDirection::CENTER};
  float targetY{0.0F};
  float errorY{0.0F};
};

struct ComputeResult {
  Action action;
  TriggerState triggerState;
  TargetState targetState;
  ServoCommand servoCommand;
  GimbalCommand gimbalCommand;
};

struct TurretStatusView {
  TargetState targetState{TargetState::TARGET_NONE};
  Action action{Action::ACTION_IDLE};
  TriggerState triggerState{TriggerState::TRIGGER_SKIP};
  float confidence{0.0F};
  float distanceM{0.0F};
};

class CoreController {
private:
  float confidenceThreshold;
  float maxDistance;

  ServoCommand computeServoCommand(float targetX);
  GimbalCommand computeGimbalCommand(float targetY);

public:
  CoreController(float confidenceThreshold, float maxDistance)
    : confidenceThreshold(confidenceThreshold)
    , maxDistance(maxDistance)
  {
  }

  ComputeResult computeTriggerResult(const antidrone_turret::TargetSample& target, antidrone_turret::ActuatorState actuatorState);
};

TurretStatusView makeTurretStatus(const antidrone_turret::TargetSample& target, const ComputeResult& result);

}  // namespace core