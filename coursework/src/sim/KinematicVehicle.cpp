#include "follow/sim/KinematicVehicle.h"

#include <cmath>

#include "follow/core/Angles.h"

namespace follow::sim {

namespace {

constexpr double kGravity = 9.81;

}  // namespace

KinematicVehicle::KinematicVehicle(const Pose& initial, const KinematicVehicleConfig& config)
  : current(initial)
  , config(config)
{
}

void KinematicVehicle::step(const std::optional<core::VelocityCmd>& command, double dtSec)
{
  core::VelocityCmd target = command.value_or(core::VelocityCmd{});
  double blend = dtSec / this->config.tauS;

  double newSpeed = this->speed + (target.vx - this->speed) * blend;
  double acceleration = (newSpeed - this->speed) / dtSec;
  this->speed = newSpeed;
  this->rate += (target.yawRate - this->rate) * blend;

  this->current.yaw = core::wrapPi(this->current.yaw + this->rate * dtSec);
  this->current.positionNed.x += this->speed * std::cos(this->current.yaw) * dtSec;
  this->current.positionNed.y += this->speed * std::sin(this->current.yaw) * dtSec;
  // Speeding up tilts the nose down.
  this->current.pitch = -std::atan2(acceleration, kGravity);
  this->current.roll = 0.0;
}

}  // namespace follow::sim
