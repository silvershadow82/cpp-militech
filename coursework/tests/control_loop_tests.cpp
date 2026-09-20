#include <gtest/gtest.h>

#include <sstream>
#include <string>
#include <vector>

#include "TestTime.h"
#include "ControlLoop.h"
#include "models/FisheyeKbModel.h"
#include "util/Channels.h"

using namespace follow;
using namespace follow::app;
using namespace follow::control;
using namespace follow::util;
using follow::test::at;
using S = models::State;

namespace {

class ControlLoopTest : public ::testing::Test {
protected:
  // Publishes a fresh heartbeat in `mode` with a level attitude sample, as the MAVLink thread would.
  void publishVehicle(double tSec, uint32_t mode)
  {
    this->vehicle.lastHeartbeat = at(tSec);
    this->vehicle.customMode = mode;
    this->vehicle.attitude.push({.t = at(tSec)});
    this->channels.vehicle.write(this->vehicle, at(tSec));
  }

  models::FisheyeKbModel camera{models::nominalFisheye(640, 480, 160.0)};
  control::VehicleState vehicle{};
  Channels channels{};
  std::ostringstream logText{};
  RunLogWriter log{logText};
  ControlLoop loop{models::Config{}, camera, models::CameraMount{}, channels, &log, at(0.0)};
};

}  // namespace

TEST_F(ControlLoopTest, NoVehicleDataMeansNoFcAndNoSetpoint)
{
  control::Outputs out = loop.tick(at(0.05));

  EXPECT_EQ(out.state, S::NoFc);
  EXPECT_FALSE(channels.setpoint.read());
}

TEST_F(ControlLoopTest, EngagePublishesLockRequestAndZeroSetpoint)
{
  // Setup
  publishVehicle(0.0, models::kModeLoiter);
  loop.tick(at(0.0));
  publishVehicle(0.05, models::kModeGuided);

  // Run
  control::Outputs out = loop.tick(at(0.05));

  // Assert
  EXPECT_EQ(out.state, S::Locking);
  std::vector<control::TrackerRequest> requests = channels.trackerRequests.drain();
  ASSERT_EQ(requests.size(), 1u);
  EXPECT_EQ(requests[0].kind, control::TrackerRequestKind::LockCenter);
  auto setpoint = channels.setpoint.read();
  ASSERT_TRUE(setpoint);
  EXPECT_EQ(setpoint->sequence, 1u);
  EXPECT_EQ(setpoint->value.vx, 0.0);
  EXPECT_EQ(setpoint->value.yawRate, 0.0);
}

TEST_F(ControlLoopTest, IdlePublishesNothing)
{
  publishVehicle(0.0, models::kModeLoiter);

  loop.tick(at(0.0));
  loop.tick(at(0.05));

  EXPECT_FALSE(channels.setpoint.read());
  EXPECT_TRUE(channels.trackerRequests.drain().empty());
}

TEST_F(ControlLoopTest, EveryTickIsLoggedWithFreshGroundTruth)
{
  // Setup
  publishVehicle(0.0, models::kModeLoiter);
  channels.truth.write({.bearingDeg = 1.5, .distanceM = 3.25}, at(0.0));

  // Run: truth is fresh at 0.1 s and 300 ms old at 0.3 s
  loop.tick(at(0.1));
  loop.tick(at(0.3));

  // Assert
  std::istringstream lines(logText.str());
  std::string header, first, second;
  std::getline(lines, header);
  std::getline(lines, first);
  std::getline(lines, second);
  EXPECT_EQ(first, "0.100,Idle,5,0,,,,,,,1.50,3.250");
  EXPECT_EQ(second, "0.300,Idle,5,0,,,,,,,,");
}

TEST_F(ControlLoopTest, PublishesTheOverlayEveryTick)
{
  control::Outputs out = this->loop.tick(at(0.0));

  auto overlay = this->channels.overlay.read();
  ASSERT_TRUE(overlay.has_value());
  EXPECT_EQ(overlay->value.state, out.state);
  EXPECT_DOUBLE_EQ(overlay->value.lockBox.w, out.overlay.lockBox.w);
}
