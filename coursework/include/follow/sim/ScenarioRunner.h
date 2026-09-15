#pragma once

#include <optional>
#include <vector>

#include "follow/core/CameraModel.h"
#include "follow/core/Config.h"
#include "follow/core/Frames.h"
#include "follow/core/Types.h"
#include "follow/sim/KinematicVehicle.h"
#include "follow/sim/SimTarget.h"
#include "follow/sim/SyntheticCamera.h"

namespace follow::sim {

struct ScenarioOptions {
  double durationS{20.0};
  double engageAtS{1.0};  // mode switches LOITER -> GUIDED; the target clock starts here
  double vehicleAltitudeM{2.0};
  double physicsDtS{0.01};
  // Control, attitude and camera run every N physics steps (20 Hz). The synthetic camera is
  // only stepped on these control ticks, so a configured camera fps above the control rate is
  // capped at it, and delivered latency rounds up to the nearest control period (e.g. 80 ms
  // configured -> 100 ms effective at 20 Hz control).
  int controlEvery{5};
  SyntheticCameraConfig camera{};
  KinematicVehicleConfig vehicle{};
};

struct StepRecord {
  double tS{0.0};
  core::State state{core::State::Idle};
  std::optional<core::VelocityCmd> setpoint{};
  bool targetValid{false};
  double trueBearingDeg{0.0};
  double trueDistanceM{0.0};  // camera to target center
};

struct ScenarioResult {
  std::vector<StepRecord> steps;
  std::vector<core::State> states;        // state sequence with consecutive duplicates removed
  std::optional<double> lockDistanceM{};  // true distance when Following was first entered
};

// Runs Core against SyntheticCamera + KinematicVehicle, starting in LOITER with the vehicle
// at the origin facing north.
ScenarioResult runScenario(const core::Config& config,
                           const core::CameraModel& camera,
                           const core::CameraMount& mount,
                           const SimTarget& target,
                           const ScenarioOptions& options);

}  // namespace follow::sim
