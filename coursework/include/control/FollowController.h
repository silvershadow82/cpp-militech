#pragma once

#include "follow/core/Config.h"
#include "follow/core/Types.h"

namespace follow::core {

// Yaw toward the target and hold distance. Call only with a valid TargetState.
class FollowController {
public:
  explicit FollowController(const ControlConfig& config)
    : config(config)
  {
  }

  VelocityCmd update(const TargetState& target, double dtSec);
  // Restarts the forward-speed slew limiter from zero.
  void reset() { this->previousVx = 0.0; }

private:
  ControlConfig config;
  double previousVx{0.0};
};

}  // namespace follow::core
