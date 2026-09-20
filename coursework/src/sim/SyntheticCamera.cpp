#include "sim/SyntheticCamera.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace follow::sim {

namespace {

bool intersects(const models::BBox& a, const models::BBox& b)
{
  return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
}

models::BBox expand(const models::BBox& box, double factor)
{
  double w = box.w * factor;
  double h = box.h * factor;
  return {.x = box.centerU() - w / 2.0, .y = box.centerV() - h / 2.0, .w = w, .h = h};
}

}  // namespace

SyntheticCamera::SyntheticCamera(const models::CameraModel& camera, const models::CameraMount& mount, const SyntheticCameraConfig& config)
  : camera(camera)
  , mount(mount)
  , config(config)
  , rng(config.seed)
{
}

std::optional<models::BBox> SyntheticCamera::project(const Pose& vehicle,
                                                     const models::Vec3& targetGroundNed,
                                                     double heightM,
                                                     double widthM) const
{
  double dx = targetGroundNed.x - vehicle.positionNed.x;
  double dy = targetGroundNed.y - vehicle.positionNed.y;
  double horizontal = std::hypot(dx, dy);
  if (horizontal < 1e-6) {
    return std::nullopt;
  }
  // The target is an upright rectangle facing the camera.
  double perpNorth = -dy / horizontal;
  double perpEast = dx / horizontal;

  double minU = std::numeric_limits<double>::max();
  double minV = std::numeric_limits<double>::max();
  double maxU = std::numeric_limits<double>::lowest();
  double maxV = std::numeric_limits<double>::lowest();
  for (double lateral : std::array{-widthM / 2.0, 0.0, widthM / 2.0}) {
    for (double up : std::array{0.0, heightM / 2.0, heightM}) {
      models::Vec3 relative{targetGroundNed.x + perpNorth * lateral - vehicle.positionNed.x,
                            targetGroundNed.y + perpEast * lateral - vehicle.positionNed.y,
                            -up - vehicle.positionNed.z};
      models::Vec3 body = models::nedToBody(relative, vehicle.roll, vehicle.pitch, vehicle.yaw);
      std::optional<models::Pixel> pixel = this->camera.rayToPixel(models::normalized(models::bodyToCamera(body, this->mount)));
      if (!pixel) {
        return std::nullopt;
      }
      minU = std::min(minU, pixel->u);
      maxU = std::max(maxU, pixel->u);
      minV = std::min(minV, pixel->v);
      maxV = std::max(maxV, pixel->v);
    }
  }

  const models::Intrinsics& k = this->camera.intrinsics();
  double left = std::max(minU, 0.0);
  double top = std::max(minV, 0.0);
  double right = std::min(maxU, static_cast<double>(k.width));
  double bottom = std::min(maxV, static_cast<double>(k.height));
  if (right <= left || bottom <= top) {
    return std::nullopt;
  }
  return models::BBox{.x = left, .y = top, .w = right - left, .h = bottom - top};
}

void SyntheticCamera::handle(const control::TrackerRequest& request, models::TimePoint now)
{
  switch (request.kind) {
    case control::TrackerRequestKind::LockCenter:
      this->locked = false;
      this->pendingLock = request.hint;
      this->reacquireHint.reset();
      break;
    case control::TrackerRequestKind::Reacquire:
      this->reacquireHint = expand(request.hint, this->config.reacquireExpand);
      this->nextReacquire = now;
      break;
    case control::TrackerRequestKind::Unlock:
      this->locked = false;
      this->pendingLock.reset();
      this->reacquireHint.reset();
      break;
    case control::TrackerRequestKind::None:
      break;
  }
}

std::optional<models::TargetObservation> SyntheticCamera::step(models::TimePoint now,
                                                               const Pose& vehicle,
                                                               const SimTarget& target,
                                                               double targetTimeS)
{
  auto period = std::chrono::duration_cast<models::Clock::duration>(std::chrono::duration<double>(1.0 / this->config.fps));
  if (!this->nextCapture) {
    this->nextCapture = now;
  }
  if (now >= *this->nextCapture) {
    this->capture(now, vehicle, target, targetTimeS);
    this->nextCapture = std::max(*this->nextCapture + period, now + period / 2);
  }

  while (!this->inFlight.empty() && this->inFlight.front().tFrame + this->config.latency <= now) {
    this->delivered = this->inFlight.front();
    this->inFlight.pop_front();
  }
  return this->delivered;
}

void SyntheticCamera::capture(models::TimePoint now, const Pose& vehicle, const SimTarget& target, double targetTimeS)
{
  std::optional<models::BBox> truth;
  if (!target.occludedAt(targetTimeS)) {
    truth = this->project(vehicle, target.positionAt(targetTimeS), target.height(), target.width());
  }

  if (this->pendingLock) {
    this->locked = truth && intersects(*truth, *this->pendingLock);
    this->pendingLock.reset();
  }
  if (!this->locked && this->reacquireHint && now >= this->nextReacquire) {
    if (truth && intersects(*truth, *this->reacquireHint)) {
      this->locked = true;
      this->reacquireHint.reset();
    }
    else {
      this->nextReacquire = now + this->config.reacquirePeriod;
    }
  }
  if (!truth) {
    this->locked = false;
  }

  models::TargetObservation observation{.tFrame = now};
  if (this->locked && truth) {
    std::normal_distribution<double> noise(0.0, this->config.pixelNoiseSigma);
    observation.box = {.x = truth->x + noise(this->rng),
                       .y = truth->y + noise(this->rng),
                       .w = truth->w + noise(this->rng),
                       .h = truth->h + noise(this->rng)};
    observation.ok = true;
    observation.confidence = 1.0;
  }
  this->inFlight.push_back(observation);
}

}  // namespace follow::sim
