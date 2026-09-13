#pragma once

#include <cstdint>
#include <optional>

#include "follow/core/AttitudeHistory.h"
#include "follow/core/CameraModel.h"
#include "follow/core/Config.h"
#include "follow/core/FollowController.h"
#include "follow/core/Frames.h"
#include "follow/core/Supervisor.h"
#include "follow/core/TargetEstimator.h"
#include "follow/core/Types.h"

namespace follow::core {

struct VehicleState {
  std::optional<TimePoint> lastHeartbeat{};
  uint32_t customMode{0};
  AttitudeHistory attitude{};
};

struct Inputs {
  TimePoint now{};
  VehicleState vehicle{};
  std::optional<TargetObservation> target{};
  std::optional<RangeMeasurement> range{};
};

enum class TrackerRequestKind { None, LockCenter, Reacquire, Unlock };

struct TrackerRequest {
  TrackerRequestKind kind{TrackerRequestKind::None};
  BBox hint{};  // LockCenter: the lock box; Reacquire: the last good bbox
};

struct OverlayInfo {
  State state{State::Idle};
  BBox lockBox{};
  std::optional<BBox> targetBox{};
};

struct Outputs {
  State state{State::Idle};
  std::optional<VelocityCmd> setpoint{};  // nullopt = send nothing
  TrackerRequest tracker{};
  OverlayInfo overlay{};
  TargetState target{};
};

// One control step. Deterministic: no clock reads, threads or I/O.
class Core {
public:
  Core(const Config& config, const CameraModel& camera, const CameraMount& mount);

  Outputs step(const Inputs& inputs);
  BBox lockBox() const;

private:
  Config config;
  const CameraModel& camera;
  TargetEstimator estimator;
  FollowController controller;
  Supervisor supervisor;
  std::optional<TimePoint> lastStep{};
};

}  // namespace follow::core
