#pragma once

#include <optional>
#include <string>
#include <vector>

#include "follow/core/Config.h"
#include "follow/core/Types.h"
#include "follow/sim/ScenarioRunner.h"

namespace follow::sim {

// Pass criteria of a scenario. Times are seconds after engage (the first Locking step); only steps
// from engage on are checked.
struct ScenarioExpect {
  enum class DistanceReference { Lock, Set };

  struct Lag {
    double fromS{0.0};
    double toS{0.0};
    double maxOverLockM{0.0};  // distance may exceed the lock distance by at most this
    double maxSpreadM{0.0};    // max - min distance inside the window
  };

  std::optional<std::vector<core::State>> states{};  // exact state sequence from engage on
  std::vector<core::State> statesInOrder{};          // must appear in this order, other states may be between
  std::optional<core::State> finalState{};
  std::optional<double> bearingMaxDeg{};  // while Following, from Following start + bearingSettleS
  double bearingSettleS{0.0};
  std::optional<double> lockDistanceTolerance{};   // every Following step within this fraction of the lock distance
  std::optional<double> finalDistanceTolerance{};  // last step within this fraction of the reference
  DistanceReference finalDistanceReference{DistanceReference::Lock};
  std::optional<double> minDistanceM{};
  std::optional<Lag> lag{};
  bool noForwardSpeed{false};
};

// Checks a run against the expectations and the command envelope (setpoint limits; no setpoint in
// Idle/NoFc; zero setpoints outside Following). toleranceScale widens tolerances, bearing and lag
// limits (1.0 for the kinematic loop, 1.5 for SITL). Returns one message per failed criterion.
std::vector<std::string> checkRun(const std::vector<StepRecord>& steps,
                                  const ScenarioExpect& expect,
                                  const core::ControlConfig& control,
                                  double toleranceScale = 1.0);

const char* stateName(core::State state);
std::optional<core::State> stateFromName(const std::string& name);

}  // namespace follow::sim
