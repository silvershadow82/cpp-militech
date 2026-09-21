#include "control/TargetEstimator.h"

#include <cmath>

#include "models/Angles.h"

namespace follow::control {

TargetEstimator::TargetEstimator(const models::EstimatorConfig &config,
                                 const interfaces::ICameraModel &camera,
                                 const models::CameraMount &mount)
  : config(config)
  , camera(camera)
  , mount(mount)
{
}

void TargetEstimator::lock(models::TimePoint now)
{
  this->reset();
  this->lockTime = now;
}

void TargetEstimator::reset()
{
  this->lockTime.reset();
  this->lastFrame.reset();
  this->previousArea.reset();
  this->lastFrameJumped = false;
  this->smoothedSize.reset();
  this->referenceSize.reset();
  this->heldRange.reset();
  this->lastGood.reset();
}

models::TargetState TargetEstimator::update(models::TimePoint now,
                                            const models::AttitudeHistory &attitude,
                                            const std::optional<models::TargetObservation> &observation,
                                            const std::optional<models::RangeMeasurement> &range)
{
  const models::TargetState invalid{};
  if (!observation || !observation->ok || observation->confidence < this->config.minConfidence) {
    return invalid;
  }
  const models::TargetObservation &obs = *observation;
  if (this->lockTime && obs.tFrame < *this->lockTime) {
    return invalid;
  }
  if (now - obs.tFrame > this->config.stale || this->touchesBorder(obs.box)) {
    return invalid;
  }
  std::optional<models::AttitudeSample> atFrame = attitude.at(obs.tFrame);
  std::optional<models::AttitudeSample> atNow = attitude.latest();
  if (!atFrame || !atNow || now - atNow->t > this->config.attitudeStale) {
    return invalid;
  }

  // Bearing of the box center in the level frame, corrected for yaw since the frame was captured.
  models::Pixel center{obs.box.centerU(), obs.box.centerV()};
  models::Vec3 level =
    models::bodyToLevel(models::cameraToBody(this->camera.pixelToRay(center), this->mount), atFrame->roll, atFrame->pitch);
  double bearingAtFrame = std::atan2(level.y, level.x);
  double bearingNow = models::wrapPi(bearingAtFrame - models::wrapPi(atNow->yaw - atFrame->yaw));

  double angularHeight =
    models::angleBetween(this->camera.pixelToRay({center.u, obs.box.y}), this->camera.pixelToRay({center.u, obs.box.y + obs.box.h}));

  // The control loop can see the same frame twice; smoothing and the jump check run once per frame.
  if (!this->lastFrame || obs.tFrame != *this->lastFrame) {
    this->lastFrame = obs.tFrame;
    double area = obs.box.area();
    this->lastFrameJumped = this->previousArea && (area > *this->previousArea * this->config.maxAreaJump ||
                                                   area * this->config.maxAreaJump < *this->previousArea);
    this->previousArea = area;
    if (!this->lastFrameJumped) {
      this->smoothedSize = this->smooth(this->smoothedSize, angularHeight);
    }
  }
  if (this->lastFrameJumped || !this->smoothedSize) {
    return invalid;
  }
  if (!this->referenceSize) {
    this->referenceSize = *this->smoothedSize;
  }

  models::TargetState state{.valid = true, .bearingRad = bearingNow, .ratio = *this->referenceSize / *this->smoothedSize};

  if (range && range->valid && std::abs(bearingNow) < models::degToRad(this->config.rangeGateDeg)) {
    auto offset = range->t - obs.tFrame;
    if (offset < models::Clock::duration::zero()) {
      offset = -offset;
    }
    if (offset <= this->config.rangeSync) {
      this->heldRange = range->rangeM;
      this->heldRangeTime = now;
    }
  }

  if (this->heldRange && now - this->heldRangeTime <= this->config.rangeHold) {
    state.distanceM = *this->heldRange;
    state.source = models::DistanceSource::Range;
  }
  else if (this->config.targetHeightM) {
    state.distanceM = *this->config.targetHeightM / (2.0 * std::tan(*this->smoothedSize / 2.0));
    state.source = models::DistanceSource::KnownSize;
  }

  this->lastGood = obs.box;
  return state;
}

bool TargetEstimator::touchesBorder(const models::BBox &box) const
{
  const models::Intrinsics &k = this->camera.intrinsics();
  double margin = this->config.borderMarginPx;
  return box.x < margin || box.y < margin || box.x + box.w > k.width - margin || box.y + box.h > k.height - margin;
}

double TargetEstimator::smooth(const std::optional<double> &previous, double sample) const
{
  return previous ? this->config.emaAlpha * sample + (1.0 - this->config.emaAlpha) * *previous : sample;
}

}  // namespace follow::control
