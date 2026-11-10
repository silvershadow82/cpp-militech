#include "sim/SimTarget.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace follow::sim {

namespace {

double durationOf(const TargetMotion& motion)
{
  return std::visit([](const auto& m) { return m.durationS; }, motion);
}

models::Vec3 advance(const models::Vec3& from, const TargetMotion& motion, double dt)
{
  if (const auto* line = std::get_if<TargetLine>(&motion)) {
    return {from.x + line->velNorth * dt, from.y + line->velEast * dt, 0.0};
  }
  if (const auto* circle = std::get_if<TargetCircle>(&motion)) {
    double dx = from.x - circle->centerNed.x;
    double dy = from.y - circle->centerNed.y;
    double radius = std::hypot(dx, dy);
    if (radius < 1e-9) {
      return from;
    }
    double phase = std::atan2(dy, dx) + circle->speed / radius * dt;
    return {circle->centerNed.x + radius * std::cos(phase), circle->centerNed.y + radius * std::sin(phase), 0.0};
  }
  return from;
}

}  // namespace

SimTarget::SimTarget(
  models::Vec3 startNed, std::vector<TargetMotion> motions, std::vector<OcclusionWindow> occlusions, double heightM, double widthM)
  : start{startNed.x, startNed.y, 0.0}
  , motions(std::move(motions))
  , occlusions(std::move(occlusions))
  , heightM(heightM)
  , widthM(widthM)
{
}

models::Vec3 SimTarget::positionAt(double tSec) const
{
  models::Vec3 position = this->start;
  double elapsed = 0.0;
  for (const TargetMotion& motion : this->motions) {
    double duration = durationOf(motion);
    double dt = std::clamp(tSec - elapsed, 0.0, duration);
    position = advance(position, motion, dt);
    if (tSec <= elapsed + duration) {
      break;
    }
    elapsed += duration;
  }
  return position;
}

bool SimTarget::occludedAt(double tSec) const
{
  return std::any_of(
    this->occlusions.begin(), this->occlusions.end(), [tSec](const OcclusionWindow& w) { return tSec >= w.startS && tSec < w.endS; });
}

}  // namespace follow::sim
