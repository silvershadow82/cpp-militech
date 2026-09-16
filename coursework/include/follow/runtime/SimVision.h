#pragma once

#include <atomic>
#include <optional>

#include "follow/config/ScenarioJson.h"
#include "follow/core/CameraModel.h"
#include "follow/core/Core.h"
#include "follow/core/Frames.h"
#include "follow/runtime/Channels.h"
#include "follow/sim/SimTarget.h"
#include "follow/sim/SyntheticCamera.h"

namespace follow::runtime {

// The vision thread of follow_app --sim: a synthetic camera over a scripted target. The target is
// placed relative to the vehicle when the first LockCenter request arrives, i.e. when the pilot engages.
class SimVision {
public:
  // `camera` must outlive this object. `durationS` is how long the scenario runs after engage.
  SimVision(const core::CameraModel& camera,
            const core::CameraMount& mount,
            const sim::SyntheticCameraConfig& cameraConfig,
            const config::TargetScript& script,
            double durationS,
            Channels& channels);

  // Handles tracker requests, then publishes an observation (when one is due) and ground truth.
  void iterate(core::TimePoint now);

  // iterate() at rateHz until `stop` is set.
  void run(const std::atomic<bool>& stop, double rateHz);

  // True once durationS has passed since engage. Safe to call from another thread.
  bool finished() const { return this->done.load(); }

  // Vehicle pose now: latest attitude, position extrapolated with its velocity (at most 200 ms).
  // nullopt until both attitude and position have been received.
  static std::optional<sim::Pose> vehiclePose(const core::VehicleState& vehicle, core::TimePoint now);

private:
  sim::SyntheticCamera camera;
  config::TargetScript script;
  double durationS;
  Channels& channels;
  std::optional<sim::SimTarget> target{};
  core::TimePoint engagedAt{};
  std::atomic<bool> done{false};
};

}  // namespace follow::runtime
