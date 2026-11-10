#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>
#include <random>

#include "Types.h"
#include "control/Core.h"
#include "interfaces/ICameraModel.h"
#include "models/Frames.h"
#include "sim/SimTarget.h"

namespace follow::sim {

struct Pose {
  models::Vec3 positionNed{};
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
  SyntheticCamera(const interfaces::ICameraModel& camera, const models::CameraMount& mount, const SyntheticCameraConfig& config);

  // Target box as the camera sees it, clipped to the image; nullopt if not visible.
  std::optional<models::BBox> project(const Pose& vehicle, const models::Vec3& targetGroundNed, double heightM, double widthM) const;

  void handle(const control::TrackerRequest& request, models::TimePoint now);

  // Captures a frame if one is due and returns the newest observation whose latency has passed.
  std::optional<models::TargetObservation> step(models::TimePoint now, const Pose& vehicle, const SimTarget& target, double targetTimeS);

private:
  void capture(models::TimePoint now, const Pose& vehicle, const SimTarget& target, double targetTimeS);

  const interfaces::ICameraModel& camera;
  models::CameraMount mount;
  SyntheticCameraConfig config;
  std::mt19937 rng;

  bool locked{false};
  std::optional<models::BBox> pendingLock{};
  std::optional<models::BBox> reacquireHint{};
  models::TimePoint nextReacquire{};
  std::optional<models::TimePoint> nextCapture{};
  std::deque<models::TargetObservation> inFlight{};
  std::optional<models::TargetObservation> delivered{};
};

}  // namespace follow::sim
