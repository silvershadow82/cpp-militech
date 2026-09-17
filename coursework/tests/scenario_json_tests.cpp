#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <nlohmann/json.hpp>
#include <numbers>
#include <string>
#include <vector>

#include "follow/config/ConfigJson.h"
#include "follow/config/ScenarioJson.h"
#include "follow/sim/ScenarioCheck.h"
#include "follow/sim/ScenarioRunner.h"

using namespace follow;
using namespace follow::config;
using nlohmann::json;
using S = core::State;

namespace {

std::string joined(const std::vector<std::string>& lines)
{
  std::string text;
  for (const std::string& line : lines) {
    text += "\n  " + line;
  }
  return text;
}

// A step with a legal setpoint for its state.
sim::StepRecord step(double tS, S state, double trueBearingDeg = 0.0, double trueDistanceM = 3.0)
{
  sim::StepRecord record{.tS = tS, .state = state, .trueBearingDeg = trueBearingDeg, .trueDistanceM = trueDistanceM};
  if (state != S::Idle && state != S::NoFc) {
    record.setpoint = core::VelocityCmd{};
  }
  return record;
}

}  // namespace

TEST(ScenarioJsonTest, ParsesTargetScriptConfigAndExpectations)
{
  // Run
  Scenario scenario = parseScenario(json::parse(R"({
    "name": "demo",
    "duration_s": 12,
    "target": {
      "start": [3.0, 1.0],
      "height_m": 1.8,
      "motions": [{"hold": {"duration_s": 2}}, {"line": {"duration_s": 5, "forward": 0.7, "right": -0.2}},
                  {"circle": {"duration_s": 60, "center": [9, 0], "speed": 1.0}}],
      "occlusions": [[5, 6]]
    },
    "config": {"control": {"enable_vx": false}},
    "expect": {
      "states": ["Locking", "Following"],
      "states_in_order": ["Following", "Lost"],
      "final_state": "Hold",
      "bearing_max_deg": 5, "bearing_settle_s": 3,
      "final_distance": {"reference": "set", "tolerance": 0.15},
      "lag": {"from_s": 13, "to_s": 21, "max_over_lock_m": 3.75, "max_spread_m": 0.3},
      "no_forward_speed": true
    }
  })"));

  // Assert
  EXPECT_EQ(scenario.name, "demo");
  EXPECT_EQ(scenario.durationS, 12.0);
  EXPECT_EQ(scenario.target.start.y, 1.0);
  EXPECT_EQ(scenario.target.heightM, 1.8);
  ASSERT_EQ(scenario.target.motions.size(), 3u);
  EXPECT_EQ(std::get<sim::TargetLine>(scenario.target.motions[1]).velEast, -0.2);
  ASSERT_EQ(scenario.target.occlusions.size(), 1u);
  EXPECT_EQ(scenario.target.occlusions[0].endS, 6.0);
  EXPECT_FALSE(parseAppConfig(scenario.configOverrides).core.control.enableVx);
  EXPECT_EQ(scenario.expect.states, (std::vector<S>{S::Locking, S::Following}));
  EXPECT_EQ(scenario.expect.statesInOrder, (std::vector<S>{S::Following, S::Lost}));
  EXPECT_EQ(scenario.expect.finalState, S::Hold);
  EXPECT_EQ(scenario.expect.bearingSettleS, 3.0);
  EXPECT_EQ(scenario.expect.finalDistanceReference, sim::ScenarioExpect::DistanceReference::Set);
  ASSERT_TRUE(scenario.expect.lag);
  EXPECT_EQ(scenario.expect.lag->maxOverLockM, 3.75);
  EXPECT_TRUE(scenario.expect.noForwardSpeed);
}

TEST(ScenarioJsonTest, ErrorsNameTheKey)
{
  auto errorOf = [](const char* text) -> std::string {
    try {
      parseScenario(json::parse(text));
    }
    catch (const ConfigError& e) {
      return e.what();
    }
    return "";
  };
  const char* base = R"("name": "x", "duration_s": 5, "target": {"start": [3, 0], "motions": [{"hold": {"duration_s": 1}}]})";

  EXPECT_EQ(errorOf((std::string("{") + base + R"(, "expect": {"final_state": "Folowing"}})").c_str()),
            "expect.final_state: unknown state 'Folowing'");
  EXPECT_EQ(errorOf(R"({"name": "x", "duration_s": 5, "target": {"start": [3, 0], "motions": [{"jump": {}}]}})"),
            "target.motions[0]: unknown motion 'jump'");
  EXPECT_EQ(errorOf(R"({"name": "x", "duration_s": 5, "target": {"start": [3, 0], "motions": []}})"),
            "target.motions: expected a non-empty array");
  EXPECT_EQ(errorOf((std::string("{") + base + R"(, "expect": {"bearing": 5}})").c_str()), "expect.bearing: unknown key");
}

TEST(ScenarioJsonTest, PlaceRotatesTheScriptIntoTheVehicleHeading)
{
  // Setup: forward 3 m then walk forward; circle center 9 m ahead; vehicle at (10, 5) facing east
  TargetScript script{.start = {3.0, 1.0, 0.0},
                      .motions = {sim::TargetLine{.durationS = 2.0, .velNorth = 1.0, .velEast = 0.0},
                                  sim::TargetCircle{.durationS = 60.0, .centerNed = {9.0, 0.0, 0.0}, .speed = 1.0}}};
  sim::Pose vehicle{.positionNed = {10.0, 5.0, -2.0}, .yaw = std::numbers::pi / 2.0};

  // Run
  sim::SimTarget placed = script.place(vehicle);

  // Assert: 3 m forward = east, 1 m right = south; walking forward moves east
  core::Vec3 start = placed.positionAt(0.0);
  EXPECT_NEAR(start.x, 9.0, 1e-9);
  EXPECT_NEAR(start.y, 8.0, 1e-9);
  EXPECT_EQ(start.z, 0.0);
  core::Vec3 walked = placed.positionAt(2.0);
  EXPECT_NEAR(walked.x, 9.0, 1e-9);
  EXPECT_NEAR(walked.y, 10.0, 1e-9);
}

TEST(ScenarioCheckTest, ReportsStateSequenceAndFinalStateMismatch)
{
  // Setup
  std::vector<sim::StepRecord> steps{step(0.0, S::Idle), step(1.0, S::Locking), step(1.5, S::Lost), step(4.5, S::Hold)};
  sim::ScenarioExpect expect{.states = std::vector<S>{S::Locking, S::Following}, .finalState = S::Following};

  // Run
  std::vector<std::string> failures = sim::checkRun(steps, expect, core::ControlConfig{});

  // Assert
  EXPECT_EQ(failures,
            (std::vector<std::string>{"states: expected Locking -> Following, got Locking -> Lost -> Hold",
                                      "final state: expected Following, got Hold"}));
}

TEST(ScenarioCheckTest, ReportsCommandEnvelopeViolations)
{
  // Setup: a setpoint in Idle, a forward speed in Lost, too much speed while following
  std::vector<sim::StepRecord> steps{step(0.0, S::Idle), step(1.0, S::Locking), step(1.05, S::Following), step(1.1, S::Lost)};
  steps[0].setpoint = core::VelocityCmd{};
  steps[2].setpoint = core::VelocityCmd{.vx = 2.0};
  steps[3].setpoint = core::VelocityCmd{.vx = 0.1};

  // Run
  std::vector<std::string> failures = sim::checkRun(steps, sim::ScenarioExpect{}, core::ControlConfig{});

  // Assert
  EXPECT_EQ(
    failures,
    (std::vector<std::string>{"t=0.00: setpoint sent in Idle", "t=1.05: vx 2.00 exceeds vx_max", "t=1.10: non-zero setpoint in Lost"}));
}

TEST(ScenarioCheckTest, ToleranceScaleWidensBearingLimit)
{
  // Setup: 6 deg bearing while following against a 5 deg limit
  std::vector<sim::StepRecord> steps{step(1.0, S::Locking), step(1.05, S::Following, 6.0)};
  sim::ScenarioExpect expect{.bearingMaxDeg = 5.0};

  // Run + Assert
  EXPECT_EQ(sim::checkRun(steps, expect, core::ControlConfig{}, 1.0),
            (std::vector<std::string>{"bearing: max 6.00 deg while following, limit 5.00"}));
  EXPECT_TRUE(sim::checkRun(steps, expect, core::ControlConfig{}, 1.5).empty());
}

TEST(ScenarioCheckTest, StepsWithoutGroundTruthAreSkipped)
{
  // Setup: a run log has no ground truth on the Locking step, before the simulated target exists
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<sim::StepRecord> steps{
    step(1.0, S::Locking, nan, nan), step(1.05, S::Following, 1.0, 3.0), step(1.1, S::Following, 1.0, 3.1)};
  sim::ScenarioExpect expect{.bearingMaxDeg = 5.0, .lockDistanceTolerance = 0.15, .minDistanceM = 1.5};

  // Run + Assert: the lock distance comes from the first step with truth, NaN never fails a check
  EXPECT_TRUE(sim::checkRun(steps, expect, core::ControlConfig{}).empty());
}

TEST(ScenarioCheckTest, RunWithoutEngageFails)
{
  EXPECT_EQ(sim::checkRun({step(0.0, S::Idle)}, sim::ScenarioExpect{}, core::ControlConfig{}),
            (std::vector<std::string>{"never engaged: no Locking step"}));
}

TEST(ScenarioJsonTest, EveryCommittedScenarioPassesInTheKinematicLoop)
{
  // Setup
  std::vector<std::filesystem::path> files;
  for (const auto& entry : std::filesystem::directory_iterator(FOLLOW_CONFIG_DIR "/scenarios")) {
    if (entry.path().extension() == ".json") {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  ASSERT_EQ(files.size(), 11u);
  core::FisheyeKbModel camera{core::nominalFisheye(640, 480, 160.0)};

  for (const std::filesystem::path& file : files) {
    // Run: engage at 1 s with the vehicle 2 m up at the origin facing north
    Scenario scenario = loadScenario(file);
    // The committed follow.json plus the scenario's overrides: the same config follow_app --sim loads,
    // so retuning follow.json cannot silently validate against the wrong envelope.
    AppConfig config = loadAppConfig(FOLLOW_CONFIG_DIR "/follow.json", scenario.configOverrides);
    sim::ScenarioOptions options{};
    options.durationS = options.engageAtS + scenario.durationS;
    sim::Pose atEngage{.positionNed = {0.0, 0.0, -options.vehicleAltitudeM}};
    sim::ScenarioResult result = sim::runScenario(config.core, camera, core::CameraMount{}, scenario.target.place(atEngage), options);

    // Assert
    std::vector<std::string> failures = sim::checkRun(result.steps, scenario.expect, config.core.control);
    EXPECT_TRUE(failures.empty()) << scenario.name << joined(failures);
    EXPECT_EQ(scenario.name, file.stem().string());
  }
}
