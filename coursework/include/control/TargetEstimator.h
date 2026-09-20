#pragma once

#include <optional>

#include "follow/core/AttitudeHistory.h"
#include "follow/core/CameraModel.h"
#include "follow/core/Config.h"
#include "follow/core/Frames.h"
#include "follow/core/Types.h"

namespace follow::core {

// Turns tracker observations (plus attitude and optional range) into a TargetState.
class TargetEstimator {
public:
  TargetEstimator(const EstimatorConfig& config, const CameraModel& camera, const CameraMount& mount);

  // Starts a new lock at `now`: frames captured earlier are ignored, and the first valid
  // observation after this becomes the size reference for TargetState::ratio.
  void lock(TimePoint now);
  // Forgets the lock, smoothing, last good box and held range.
  void reset();

  TargetState update(TimePoint now,
                     const AttitudeHistory& attitude,
                     const std::optional<TargetObservation>& observation,
                     const std::optional<RangeMeasurement>& range);

  std::optional<BBox> lastGoodBox() const { return this->lastGood; }

private:
  bool touchesBorder(const BBox& box) const;
  double smooth(const std::optional<double>& previous, double sample) const;

  EstimatorConfig config;
  const CameraModel& camera;
  CameraMount mount;

  std::optional<TimePoint> lockTime{};
  std::optional<TimePoint> lastFrame{};
  std::optional<double> previousArea{};
  bool lastFrameJumped{false};
  std::optional<double> smoothedSize{};  // angular height, rad
  std::optional<double> referenceSize{};
  std::optional<double> heldRange{};
  TimePoint heldRangeTime{};
  std::optional<BBox> lastGood{};
};

}  // namespace follow::core
