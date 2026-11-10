#include <gtest/gtest.h>

#include <common/mavlink.h>
#include <vector>

#include "MavlinkTestSupport.h"
#include "TestTime.h"
#include "comms/MavlinkIo.h"
#include "util/Channels.h"

using namespace follow;
using namespace follow::comms;
using namespace follow::util;
using follow::test::at;
using follow::test::decodeFrames;
using follow::test::FakeLink;
using follow::test::Peer;

namespace {

int countSent(const FakeLink& link, uint32_t messageId)
{
  int count = 0;
  for (const mavlink_message_t& message : decodeFrames(link.sent)) {
    count += message.msgid == messageId ? 1 : 0;
  }
  return count;
}

class MavlinkIoTest : public ::testing::Test {
protected:
  FakeLink link;
  Peer fc{1, 1};
  Channels channels;
  MavlinkIo io{link, comms::MavlinkIds{}, channels};
};

}  // namespace

TEST_F(MavlinkIoTest, PublishesVehicleStateWhenTheFcSpeaks)
{
  // Run + Assert: nothing to publish before the FC is heard
  io.iterate(at(1.0));
  EXPECT_FALSE(channels.vehicle.read());

  link.feed(fc.heartbeat(models::kModeGuided, true));
  io.iterate(at(1.1));

  auto vehicle = channels.vehicle.read();
  ASSERT_TRUE(vehicle);
  EXPECT_EQ(vehicle->t, at(1.1));
  EXPECT_EQ(vehicle->value.customMode, models::kModeGuided);
  EXPECT_EQ(vehicle->value.lastHeartbeat, at(1.1));
}

TEST_F(MavlinkIoTest, SendsEachNewSetpointExactlyOnce)
{
  // Run
  channels.setpoint.write({.vx = 0.5}, at(1.0));
  io.iterate(at(1.0));
  io.iterate(at(1.005));
  int afterFirst = countSent(link, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);
  channels.setpoint.write({.vx = 0.6}, at(1.05));
  io.iterate(at(1.05));

  // Assert
  EXPECT_EQ(afterFirst, 1);
  EXPECT_EQ(countSent(link, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED), 2);
}

TEST_F(MavlinkIoTest, HeartbeatIsSentBeforeTheFcIsHeard)
{
  io.iterate(at(0.0));

  EXPECT_EQ(countSent(link, MAVLINK_MSG_ID_HEARTBEAT), 1);
}
