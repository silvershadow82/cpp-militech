#include "follow/core/FollowController.h"

#include <algorithm>
#include <cmath>

#include "follow/core/Angles.h"

namespace follow::core {

VelocityCmd FollowController::update(const TargetState& target, double dtSec)
{
  double bearingDeg = std::abs(radToDeg(target.bearingRad));

  double yawRateMax = degToRad(this->config.yawRateMaxDps);
  double yawRate =
    bearingDeg < this->config.yawDeadbandDeg ? 0.0 : std::clamp(this->config.kYaw * target.bearingRad, -yawRateMax, yawRateMax);

  double error = target.distanceM ? *target.distanceM - this->config.dSet : this->config.dNominal * (target.ratio - 1.0);
  double vx =
    std::abs(error) < this->config.distDeadbandM ? 0.0 : std::clamp(this->config.kD * error, -this->config.vxMax, this->config.vxMax);

  // Turn first, then close distance.
  vx *= std::max(0.0, 1.0 - bearingDeg / this->config.headingGateDeg);

  double maxStep = this->config.vxSlew * dtSec;
  vx = std::clamp(vx, this->previousVx - maxStep, this->previousVx + maxStep);

  // Applied after the slew limiter so it is a hard limit.
  double estimatedDistance = target.distanceM ? *target.distanceM : this->config.dNominal * target.ratio;
  if (estimatedDistance < this->config.dMin) {
    vx = std::min(vx, 0.0);
  }
  if (!this->config.enableVx) {
    vx = 0.0;
  }

  this->previousVx = vx;
  return {.vx = vx, .yawRate = yawRate};
}

}  // namespace follow::core
