#include "sim/KinematicVehicle.h"

#include <algorithm>
#include <cmath>

#include "models/Angles.h"

namespace follow::sim {

namespace {

constexpr double kGravity = 9.81;

}  // namespace

KinematicVehicle::KinematicVehicle(const Pose& initial, const KinematicVehicleConfig& config)
  : current(initial)
  , config(config)
{
}

void KinematicVehicle::step(const std::optional<models::VelocityCmd>& command, double dtSec)
{
  if (dtSec <= 0.0) {
    return;
  }
  models::VelocityCmd target = command.value_or(models::VelocityCmd{});
  // Clamped so a step longer than the time constant settles on the command instead of overshooting.
  double blend = std::min(dtSec / this->config.tauS, 1.0);

  double newSpeed = this->speed + (target.vx - this->speed) * blend;
  double acceleration = (newSpeed - this->speed) / dtSec;
  this->speed = newSpeed;
  this->rate += (target.yawRate - this->rate) * blend;

  this->current.yaw = models::wrapPi(this->current.yaw + this->rate * dtSec);
  this->current.positionNed.x += this->speed * std::cos(this->current.yaw) * dtSec;
  this->current.positionNed.y += this->speed * std::sin(this->current.yaw) * dtSec;
  // Speeding up tilts the nose down.
  this->current.pitch = -std::atan2(acceleration, kGravity);
  this->current.roll = 0.0;
}

}  // namespace follow::sim
