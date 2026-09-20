#include <gtest/gtest.h>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "StatCollector.h"

using namespace follow;
using namespace follow::util;
using S = models::State;

TEST(RunLogTest, WritesHeaderAndOneLinePerRow)
{
  // Setup
  std::ostringstream out;
  RunLogWriter writer(out);

  // Run: idle before engage, then following with a metric distance and ground truth
  writer.write({.tS = 0.05, .state = S::Idle, .customMode = 5, .trueBearingDeg = 0.0, .trueDistanceM = 3.2});
  writer.write({.tS = 1.5,
                .state = S::Following,
                .customMode = 4,
                .target = {.valid = true, .bearingRad = 0.1, .ratio = 1.05, .distanceM = 3.15, .source = models::DistanceSource::KnownSize},
                .setpoint = models::VelocityCmd{.vx = 0.25, .yawRate = -0.125},
                .trueBearingDeg = 5.5,
                .trueDistanceM = 3.1234});

  // Assert
  EXPECT_EQ(out.str(),
            "t,state,mode,valid,bearing_deg,ratio,distance_m,source,vx,yaw_rate,true_bearing_deg,true_distance_m\n"
            "0.050,Idle,5,0,,,,,,,0.00,3.200\n"
            "1.500,Following,4,1,5.73,1.050,3.15,KnownSize,0.250000,-0.125000,5.50,3.123\n");
}

TEST(RunLogTest, ReaderReturnsStepsWrittenByWriter)
{
  // Setup
  std::stringstream log;
  RunLogWriter writer(log);
  writer.write(
    {.tS = 1.0, .state = S::Locking, .customMode = 4, .setpoint = models::VelocityCmd{}, .trueBearingDeg = -1.5, .trueDistanceM = 3.0});
  writer.write({.tS = 1.05,
                .state = S::Following,
                .customMode = 4,
                .target = {.valid = true},
                .setpoint = models::VelocityCmd{.vx = 0.5, .yawRate = 0.785398},
                .trueBearingDeg = 2.0,
                .trueDistanceM = 2.9});

  // Run
  std::vector<sim::StepRecord> steps = readRunLog(log);

  // Assert
  ASSERT_EQ(steps.size(), 2u);
  EXPECT_EQ(steps[0].state, S::Locking);
  ASSERT_TRUE(steps[0].setpoint);
  EXPECT_EQ(steps[0].setpoint->vx, 0.0);
  EXPECT_FALSE(steps[0].targetValid);
  EXPECT_DOUBLE_EQ(steps[1].tS, 1.05);
  EXPECT_TRUE(steps[1].targetValid);
  EXPECT_DOUBLE_EQ(steps[1].setpoint->yawRate, 0.785398);
  EXPECT_DOUBLE_EQ(steps[1].trueBearingDeg, 2.0);
  EXPECT_DOUBLE_EQ(steps[1].trueDistanceM, 2.9);
}

TEST(RunLogTest, ReaderRejectsMalformedLogs)
{
  const std::string header = "t,state,mode,valid,bearing_deg,ratio,distance_m,source,vx,yaw_rate,true_bearing_deg,true_distance_m\n";
  auto errorOf = [](const std::string& text) -> std::string {
    std::istringstream in(text);
    try {
      readRunLog(in);
    }
    catch (const std::runtime_error& e) {
      return e.what();
    }
    return "";
  };

  EXPECT_EQ(errorOf("time,state\n").rfind("run log: missing header", 0), 0u);
  EXPECT_EQ(errorOf(header + "1.0,Flying,4,0,,,,,,,0.0,3.0\n"), "run log line 2: unknown state 'Flying'");
  EXPECT_EQ(errorOf(header + "1.0,Idle,4\n"), "run log line 2: expected 12 fields, got 3");
  EXPECT_EQ(errorOf(header + "1.0,Idle,4,0,,,,,abc,0,0.0,3.0\n"), "run log line 2: bad vx 'abc'");
}

TEST(RunLogTest, StepsWithoutGroundTruthReadAsNan)
{
  std::istringstream in(
    "t,state,mode,valid,bearing_deg,ratio,distance_m,source,vx,yaw_rate,true_bearing_deg,true_distance_m\n"
    "0.050,Idle,5,0,,,,,,,,\n");

  std::vector<sim::StepRecord> steps = readRunLog(in);

  ASSERT_EQ(steps.size(), 1u);
  EXPECT_TRUE(std::isnan(steps[0].trueBearingDeg));
  EXPECT_TRUE(std::isnan(steps[0].trueDistanceM));
}
