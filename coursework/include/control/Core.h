#pragma once

#include "Types.h"
#include "control/FollowController.h"
#include "control/Supervisor.h"
#include "control/TargetEstimator.h"
#include "interfaces/ICameraModel.h"
#include "models/AttitudeHistory.h"
#include "models/Config.h"
#include "models/Frames.h"

#include <cstdint>
#include <optional>

namespace follow::control
{

  struct VehicleState
  {
    std::optional<models::TimePoint> lastHeartbeat{};
    uint32_t customMode{0};
    bool armed{false};
    models::AttitudeHistory attitude{};
    std::optional<models::LocalPositionNed> position{}; // not used by Core; the simulated camera needs it
  };

  struct Inputs
  {
    models::TimePoint now{};
    VehicleState vehicle{};
    std::optional<models::TargetObservation> target{};
    std::optional<models::RangeMeasurement> range{};
  };

  enum class TrackerRequestKind
  {
    None,
    LockCenter,
    Reacquire,
    Unlock
  };

  struct TrackerRequest
  {
    TrackerRequestKind kind{TrackerRequestKind::None};
    models::BBox hint{}; // LockCenter: the lock box; Reacquire: the last good bbox
  };

  struct OverlayInfo
  {
    models::State state{models::State::Idle};
    models::BBox lockBox{};
    std::optional<models::BBox> targetBox{};
  };

  struct Outputs
  {
    models::State state{models::State::Idle};
    std::optional<models::VelocityCmd> setpoint{}; // nullopt = send nothing
    TrackerRequest tracker{};
    OverlayInfo overlay{};
    models::TargetState target{};
  };

  // One control step. Deterministic: no clock reads, threads or I/O.
  class Core
  {
  public:
    Core(const models::Config &config, const interfaces::ICameraModel &camera, const models::CameraMount &mount);

    Outputs step(const Inputs &inputs);
    models::BBox lockBox() const;

  private:
    models::Config config;
    const interfaces::ICameraModel &camera;
    TargetEstimator estimator;
    FollowController controller;
    Supervisor supervisor;
    std::optional<models::TimePoint> lastStep{};
  };

} // namespace follow::control
