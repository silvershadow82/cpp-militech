#pragma once

#include <atomic>
#include <optional>

#include "config/ScenarioLoader.h"
#include "control/Core.h"
#include "interfaces/ICameraModel.h"
#include "models/Frames.h"
#include "sim/SimTarget.h"
#include "sim/SyntheticCamera.h"
#include "util/Channels.h"

namespace follow::providers {

// The vision thread of follow_app --sim: a synthetic camera over a scripted target. The target is
// placed relative to the vehicle when the first LockCenter request arrives, i.e. when the pilot engages.
class SimVision {
public:
  // `camera` must outlive this object. `durationS` is how long the scenario runs after engage.
  SimVision(const interfaces::ICameraModel& camera,
            const models::CameraMount& mount,
            const sim::SyntheticCameraConfig& cameraConfig,
            const config::TargetScript& script,
            double durationS,
            util::Channels& channels);

  // Handles tracker requests, then publishes an observation (when one is due) and ground truth.
  void iterate(models::TimePoint now);

  // iterate() at rateHz until `stop` is set.
  void run(const std::atomic<bool>& stop, double rateHz);

  // True once durationS has passed since engage. Safe to call from another thread.
  bool finished() const { return this->done.load(); }

  // Vehicle pose now: latest attitude, position extrapolated with its velocity (at most 200 ms).
  // nullopt until both attitude and position have been received.
  static std::optional<sim::Pose> vehiclePose(const control::VehicleState& vehicle, models::TimePoint now);

private:
  sim::SyntheticCamera camera;
  config::TargetScript script;
  double durationS;
  util::Channels& channels;
  std::optional<sim::SimTarget> target{};
  models::TimePoint engagedAt{};
  bool lockPending{false};  // a LockCenter arrived before a pose was available; place on the first pose after
  std::atomic<bool> done{false};
};

}  // namespace follow::providers
