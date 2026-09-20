#pragma once

#include <optional>

#include "Types.h"
#include "models/AttitudeHistory.h"
#include "models/CameraModel.h"
#include "models/Config.h"
#include "models/Frames.h"

namespace follow::control {

// Turns tracker observations (plus attitude and optional range) into a TargetState.
class TargetEstimator {
public:
  TargetEstimator(const models::EstimatorConfig& config, const models::CameraModel& camera, const models::CameraMount& mount);

  // Starts a new lock at `now`: frames captured earlier are ignored, and the first valid
  // observation after this becomes the size reference for TargetState::ratio.
  void lock(models::TimePoint now);
  // Forgets the lock, smoothing, last good box and held range.
  void reset();

  models::TargetState update(models::TimePoint now,
                             const models::AttitudeHistory& attitude,
                             const std::optional<models::TargetObservation>& observation,
                             const std::optional<models::RangeMeasurement>& range);

  std::optional<models::BBox> lastGoodBox() const { return this->lastGood; }

private:
  bool touchesBorder(const models::BBox& box) const;
  double smooth(const std::optional<double>& previous, double sample) const;

  models::EstimatorConfig config;
  const models::CameraModel& camera;
  models::CameraMount mount;

  std::optional<models::TimePoint> lockTime{};
  std::optional<models::TimePoint> lastFrame{};
  std::optional<double> previousArea{};
  bool lastFrameJumped{false};
  std::optional<double> smoothedSize{};  // angular height, rad
  std::optional<double> referenceSize{};
  std::optional<double> heldRange{};
  models::TimePoint heldRangeTime{};
  std::optional<models::BBox> lastGood{};
};

}  // namespace follow::control
