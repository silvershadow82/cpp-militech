#pragma once

#include "Types.h"
#include "models/Config.h"

namespace follow::control {

// Yaw toward the target and hold distance. Call only with a valid TargetState.
class FollowController {
public:
  explicit FollowController(const models::ControlConfig& config)
    : config(config)
  {
  }

  models::VelocityCmd update(const models::TargetState& target, double dtSec);
  // Restarts the forward-speed slew limiter from zero.
  void reset() { this->previousVx = 0.0; }

private:
  models::ControlConfig config;
  double previousVx{0.0};
};

}  // namespace follow::control
