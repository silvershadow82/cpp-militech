#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>
#include <random>

#include "follow/core/CameraModel.h"
#include "follow/core/Core.h"
#include "follow/core/Frames.h"
#include "follow/core/Types.h"
#include "follow/sim/SimTarget.h"

namespace follow::sim {

struct Pose {
  core::Vec3 positionNed{};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

struct SyntheticCameraConfig {
  double fps{20.0};
  std::chrono::milliseconds latency{80};
  double pixelNoiseSigma{1.5};
  std::chrono::milliseconds reacquirePeriod{500};
  double reacquireExpand{1.5};
  uint32_t seed{42};
};

// Stands in for camera + tracker: projects the simulated target and emits observations
// with frame rate, latency, pixel noise and tracker lock/loss behaviour.
class SyntheticCamera {
public:
  SyntheticCamera(const core::CameraModel& camera, const core::CameraMount& mount, const SyntheticCameraConfig& config);

  // Target box as the camera sees it, clipped to the image; nullopt if not visible.
  std::optional<core::BBox> project(const Pose& vehicle, const core::Vec3& targetGroundNed, double heightM, double widthM) const;

  void handle(const core::TrackerRequest& request, core::TimePoint now);

  // Captures a frame if one is due and returns the newest observation whose latency has passed.
  std::optional<core::TargetObservation> step(core::TimePoint now, const Pose& vehicle, const SimTarget& target, double targetTimeS);

private:
  void capture(core::TimePoint now, const Pose& vehicle, const SimTarget& target, double targetTimeS);

  const core::CameraModel& camera;
  core::CameraMount mount;
  SyntheticCameraConfig config;
  std::mt19937 rng;

  bool locked{false};
  std::optional<core::BBox> pendingLock{};
  std::optional<core::BBox> reacquireHint{};
  core::TimePoint nextReacquire{};
  std::optional<core::TimePoint> nextCapture{};
  std::deque<core::TargetObservation> inFlight{};
  std::optional<core::TargetObservation> delivered{};
};

}  // namespace follow::sim
