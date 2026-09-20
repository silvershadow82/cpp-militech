#pragma once

#include <optional>

#include "Types.h"
#include "sim/SyntheticCamera.h"

namespace follow::sim {

struct KinematicVehicleConfig {
  double tauS{0.3};  // first-order response of forward speed and yaw rate
};

// Minimal multicopter stand-in: forward speed and yaw rate follow the command with a lag,
// pitch follows forward acceleration, altitude is held.
class KinematicVehicle {
public:
  KinematicVehicle(const Pose& initial, const KinematicVehicleConfig& config);

  // nullopt behaves like a zero command (the FC holding position).
  void step(const std::optional<models::VelocityCmd>& command, double dtSec);

  const Pose& pose() const { return this->current; }
  double forwardSpeed() const { return this->speed; }
  double yawRate() const { return this->rate; }

private:
  Pose current;
  KinematicVehicleConfig config;
  double speed{0.0};
  double rate{0.0};
};

}  // namespace follow::sim
