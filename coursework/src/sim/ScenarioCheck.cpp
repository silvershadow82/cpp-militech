#include "follow/sim/ScenarioCheck.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "follow/core/Angles.h"

namespace follow::sim {

namespace {

constexpr std::array kAllStates{
  core::State::Idle, core::State::Locking, core::State::Following, core::State::Lost, core::State::Hold, core::State::NoFc};

std::string number(double value)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(2) << value;
  return out.str();
}

std::string sequenceText(const std::vector<core::State>& states)
{
  std::string text;
  for (core::State state : states) {
    text += (text.empty() ? "" : " -> ") + std::string(stateName(state));
  }
  return text.empty() ? "<none>" : text;
}

bool containsInOrder(const std::vector<core::State>& sequence, const std::vector<core::State>& wanted)
{
  auto next = sequence.begin();
  for (core::State state : wanted) {
    next = std::find(next, sequence.end(), state);
    if (next == sequence.end()) {
      return false;
    }
    ++next;
  }
  return true;
}

// A run log has no ground truth for steps before the simulated target exists (NaN).
bool hasTruth(const StepRecord& step)
{
  return !std::isnan(step.trueBearingDeg) && !std::isnan(step.trueDistanceM);
}

// Collects failures, keeping only the first occurrence of each rule so a long run stays readable.
class Failures {
public:
  void add(const std::string& rule, const std::string& message)
  {
    if (std::find(this->rules.begin(), this->rules.end(), rule) == this->rules.end()) {
      this->rules.push_back(rule);
      this->messages.push_back(message);
    }
  }

  std::vector<std::string> messages;

private:
  std::vector<std::string> rules;
};

}  // namespace

const char* stateName(core::State state)
{
  switch (state) {
    case core::State::Idle:
      return "Idle";
    case core::State::Locking:
      return "Locking";
    case core::State::Following:
      return "Following";
    case core::State::Lost:
      return "Lost";
    case core::State::Hold:
      return "Hold";
    case core::State::NoFc:
      return "NoFc";
  }
  return "?";
}

std::optional<core::State> stateFromName(const std::string& name)
{
  for (core::State state : kAllStates) {
    if (name == stateName(state)) {
      return state;
    }
  }
  return std::nullopt;
}

std::vector<std::string> checkRun(const std::vector<StepRecord>& steps,
                                  const ScenarioExpect& expect,
                                  const core::ControlConfig& control,
                                  double toleranceScale)
{
  Failures failures;
  const double yawRateMax = core::degToRad(control.yawRateMaxDps);

  // Command envelope over the whole run.
  for (const StepRecord& step : steps) {
    std::string at = "t=" + number(step.tS) + ": ";
    bool silent = step.state == core::State::Idle || step.state == core::State::NoFc;
    if (silent && step.setpoint) {
      failures.add("silent", at + "setpoint sent in " + stateName(step.state));
    }
    if (!silent && !step.setpoint) {
      failures.add("missing", at + "no setpoint in " + stateName(step.state));
    }
    if (!step.setpoint) {
      continue;
    }
    if (std::abs(step.setpoint->vx) > control.vxMax + 1e-6) {
      failures.add("vx_max", at + "vx " + number(step.setpoint->vx) + " exceeds vx_max");
    }
    if (std::abs(step.setpoint->yawRate) > yawRateMax + 1e-6) {
      failures.add("yaw_rate_max", at + "yaw rate " + number(step.setpoint->yawRate) + " rad/s exceeds the limit");
    }
    if (step.state != core::State::Following && (step.setpoint->vx != 0.0 || step.setpoint->yawRate != 0.0)) {
      failures.add("zero", at + "non-zero setpoint in " + stateName(step.state));
    }
    if (expect.noForwardSpeed && step.setpoint->vx != 0.0) {
      failures.add("no_forward_speed", at + "forward speed " + number(step.setpoint->vx) + " commanded");
    }
  }

  auto engage = std::find_if(steps.begin(), steps.end(), [](const StepRecord& s) { return s.state == core::State::Locking; });
  if (engage == steps.end()) {
    failures.add("engage", "never engaged: no Locking step");
    return failures.messages;
  }
  const double engageS = engage->tS;
  std::vector<StepRecord> run(engage, steps.end());

  std::vector<core::State> sequence;
  for (const StepRecord& step : run) {
    if (sequence.empty() || sequence.back() != step.state) {
      sequence.push_back(step.state);
    }
  }
  if (expect.states && sequence != *expect.states) {
    failures.add("states", "states: expected " + sequenceText(*expect.states) + ", got " + sequenceText(sequence));
  }
  if (!containsInOrder(sequence, expect.statesInOrder)) {
    failures.add("states_in_order", "states: expected " + sequenceText(expect.statesInOrder) + " in order, got " + sequenceText(sequence));
  }
  if (expect.finalState && sequence.back() != *expect.finalState) {
    failures.add("final_state",
                 std::string("final state: expected ") + stateName(*expect.finalState) + ", got " + stateName(sequence.back()));
  }

  // The lock distance is the true distance at the first Following step with ground truth.
  auto following =
    std::find_if(run.begin(), run.end(), [](const StepRecord& s) { return s.state == core::State::Following && hasTruth(s); });
  bool needsFollowing = expect.bearingMaxDeg || expect.lockDistanceTolerance || expect.lag ||
                        (expect.finalDistanceTolerance && expect.finalDistanceReference == ScenarioExpect::DistanceReference::Lock);
  if (following == run.end()) {
    if (needsFollowing) {
      failures.add("following", "never reached Following");
    }
    return failures.messages;
  }
  const double followingS = following->tS;
  const double lockDistance = following->trueDistanceM;

  if (expect.bearingMaxDeg) {
    double limit = *expect.bearingMaxDeg * toleranceScale;
    double worst = 0.0;
    for (const StepRecord& step : run) {
      if (step.state == core::State::Following && hasTruth(step) && step.tS >= followingS + expect.bearingSettleS) {
        worst = std::max(worst, std::abs(step.trueBearingDeg));
      }
    }
    if (worst >= limit) {
      failures.add("bearing", "bearing: max " + number(worst) + " deg while following, limit " + number(limit));
    }
  }

  if (expect.lockDistanceTolerance) {
    double allowed = *expect.lockDistanceTolerance * toleranceScale * lockDistance;
    for (const StepRecord& step : run) {
      if (step.state == core::State::Following && hasTruth(step) && std::abs(step.trueDistanceM - lockDistance) > allowed) {
        failures.add("lock_distance",
                     "t=" + number(step.tS) + ": distance " + number(step.trueDistanceM) + " m, lock distance " + number(lockDistance) +
                       " m +/- " + number(allowed));
      }
    }
  }

  if (expect.finalDistanceTolerance) {
    bool toLock = expect.finalDistanceReference == ScenarioExpect::DistanceReference::Lock;
    double reference = toLock ? lockDistance : control.dSet;
    double allowed = *expect.finalDistanceTolerance * toleranceScale * reference;
    auto last = std::find_if(run.rbegin(), run.rend(), hasTruth);
    if (last == run.rend() || std::abs(last->trueDistanceM - reference) > allowed) {
      failures.add("final_distance",
                   "final distance " + (last == run.rend() ? std::string("unknown") : number(last->trueDistanceM) + " m") + ", expected " +
                     number(reference) + " m +/- " + number(allowed));
    }
  }

  if (expect.minDistanceM) {
    for (const StepRecord& step : run) {
      if (hasTruth(step) && step.trueDistanceM < *expect.minDistanceM) {
        failures.add(
          "min_distance",
          "t=" + number(step.tS) + ": distance " + number(step.trueDistanceM) + " m inside " + number(*expect.minDistanceM) + " m");
      }
    }
  }

  if (expect.lag) {
    std::vector<double> window;
    for (const StepRecord& step : run) {
      double sinceEngage = step.tS - engageS;
      if (hasTruth(step) && sinceEngage >= expect.lag->fromS && sinceEngage <= expect.lag->toS) {
        window.push_back(step.trueDistanceM);
      }
    }
    if (window.empty()) {
      failures.add("lag", "lag: no steps between " + number(expect.lag->fromS) + " and " + number(expect.lag->toS) + " s after engage");
    }
    else {
      auto [low, high] = std::minmax_element(window.begin(), window.end());
      double maxOver = expect.lag->maxOverLockM * toleranceScale;
      double maxSpread = expect.lag->maxSpreadM * toleranceScale;
      if (*high >= lockDistance + maxOver) {
        failures.add("lag_over", "lag: distance reached " + number(*high) + " m, limit " + number(lockDistance + maxOver));
      }
      if (*high - *low >= maxSpread) {
        failures.add("lag_spread", "lag: distance spread " + number(*high - *low) + " m, limit " + number(maxSpread));
      }
    }
  }

  return failures.messages;
}

}  // namespace follow::sim
