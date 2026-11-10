#include <gtest/gtest.h>

#include "TestTime.h"
#include "models/AttitudeHistory.h"

using namespace follow::models;
using follow::test::at;

TEST(AttitudeHistory, EmptyHistoryHasNoAttitude)
{
  AttitudeHistory history;
  EXPECT_FALSE(history.latest());
  EXPECT_FALSE(history.at(at(0.0)));
}

TEST(AttitudeHistory, InterpolatesBetweenSamples)
{
  // Setup
  AttitudeHistory history;
  history.push({.t = at(0.0), .roll = 0.0, .pitch = 0.0, .yaw = 0.0});
  history.push({.t = at(0.1), .roll = 0.2, .pitch = -0.1, .yaw = 0.4});

  // Run
  std::optional<AttitudeSample> mid = history.at(at(0.05));

  // Assert
  ASSERT_TRUE(mid);
  EXPECT_NEAR(mid->roll, 0.1, 1e-9);
  EXPECT_NEAR(mid->pitch, -0.05, 1e-9);
  EXPECT_NEAR(mid->yaw, 0.2, 1e-9);
}

TEST(AttitudeHistory, InterpolatesYawAcrossWrap)
{
  // Setup: 3.0 rad to -3.1 rad is a short 0.183 rad turn through +pi
  AttitudeHistory history;
  history.push({.t = at(0.0), .yaw = 3.0});
  history.push({.t = at(0.1), .yaw = -3.1});

  // Run
  std::optional<AttitudeSample> sample = history.at(at(0.09));

  // Assert: 3.0 + 0.9 * 0.1831853 = 3.1648668, wrapped to -3.1183185
  ASSERT_TRUE(sample);
  EXPECT_NEAR(sample->yaw, -3.1183185, 1e-6);
}

TEST(AttitudeHistory, TimeAfterNewestReturnsNewest)
{
  AttitudeHistory history;
  history.push({.t = at(0.0), .yaw = 0.1});
  history.push({.t = at(0.1), .yaw = 0.2});

  std::optional<AttitudeSample> sample = history.at(at(5.0));

  ASSERT_TRUE(sample);
  EXPECT_DOUBLE_EQ(sample->yaw, 0.2);
}

TEST(AttitudeHistory, DropsSamplesOlderThanSpan)
{
  // Setup: 1 s span, samples from 0 to 2 s
  AttitudeHistory history(std::chrono::milliseconds{1000});
  for (int i = 0; i <= 20; ++i) {
    history.push({.t = at(i * 0.1), .yaw = i * 0.01});
  }

  // Assert
  EXPECT_FALSE(history.at(at(0.5)));
  EXPECT_TRUE(history.at(at(1.5)));
}

TEST(AttitudeHistory, IgnoresOutOfOrderSamples)
{
  AttitudeHistory history;
  history.push({.t = at(1.0), .yaw = 1.0});
  history.push({.t = at(0.5), .yaw = 5.0});

  ASSERT_TRUE(history.latest());
  EXPECT_DOUBLE_EQ(history.latest()->yaw, 1.0);
}
