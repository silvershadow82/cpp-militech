#include <gtest/gtest.h>

#include <common/mavlink.h>
#include <string>
#include <vector>

#include "MavlinkTestSupport.h"
#include "TestTime.h"
#include "Types.h"
#include "comms/MavlinkClient.h"

using namespace follow;
using namespace follow::comms;
using follow::test::at;
using follow::test::decodeFrames;
using follow::test::FakeLink;
using follow::test::Peer;

namespace {

class MavlinkClientTest : public ::testing::Test {
protected:
  FakeLink link;
  Peer fc{1, 1};
  MavlinkClient client{link, MavlinkIds{}};
};

}  // namespace

TEST_F(MavlinkClientTest, HeartbeatFromAutopilotSetsModeArmedAndTime)
{
  // Setup
  link.feed(fc.heartbeat(models::kModeGuided, true));

  // Run
  int accepted = client.poll(at(2.0));

  // Assert
  EXPECT_EQ(accepted, 1);
  EXPECT_EQ(client.vehicle().lastHeartbeat, at(2.0));
  EXPECT_EQ(client.vehicle().customMode, models::kModeGuided);
  EXPECT_TRUE(client.vehicle().armed);
}

TEST_F(MavlinkClientTest, MessagesFromOtherSystemsAndComponentsAreIgnored)
{
  // Setup: a GCS (sysid 255) and another onboard component of the vehicle (1/100)
  Peer gcs(255, 190);
  Peer camera(1, 100);
  link.feed(gcs.heartbeat(models::kModeGuided, true));
  link.feed(camera.heartbeat(models::kModeGuided, true));
  link.feed(camera.attitude(0.1F, 0.2F, 0.3F));

  // Run
  int accepted = client.poll(at(2.0));

  // Assert
  EXPECT_EQ(accepted, 0);
  EXPECT_FALSE(client.vehicle().lastHeartbeat);
  EXPECT_FALSE(client.vehicle().attitude.latest());
}

TEST_F(MavlinkClientTest, AttitudeIsStampedWithReceiveTime)
{
  // Setup
  link.feed(fc.attitude(0.1F, -0.2F, 1.5F));

  // Run
  client.poll(at(3.0));

  // Assert
  auto sample = client.vehicle().attitude.latest();
  ASSERT_TRUE(sample);
  EXPECT_EQ(sample->t, at(3.0));
  EXPECT_FLOAT_EQ(sample->roll, 0.1F);
  EXPECT_FLOAT_EQ(sample->pitch, -0.2F);
  EXPECT_FLOAT_EQ(sample->yaw, 1.5F);
}

TEST_F(MavlinkClientTest, NewestAttitudeOfOneBatchWins)
{
  link.feed(fc.attitude(0.0F, 0.0F, 1.0F));
  link.feed(fc.attitude(0.0F, 0.0F, 2.0F));

  client.poll(at(3.0));

  EXPECT_FLOAT_EQ(client.vehicle().attitude.latest()->yaw, 2.0F);
}

TEST_F(MavlinkClientTest, LocalPositionIsStored)
{
  link.feed(fc.localPosition(1.0F, 2.0F, -3.0F, 0.5F, -0.5F, 0.1F));

  client.poll(at(4.0));

  ASSERT_TRUE(client.vehicle().position);
  EXPECT_EQ(client.vehicle().position->t, at(4.0));
  EXPECT_FLOAT_EQ(client.vehicle().position->position.z, -3.0F);
  EXPECT_FLOAT_EQ(client.vehicle().position->velocity.y, -0.5F);
}

TEST_F(MavlinkClientTest, StatusTextReachesHandler)
{
  // Setup
  std::vector<std::string> texts;
  MavlinkClient withLog(link, MavlinkIds{}, [&texts](const std::string& text) { texts.push_back(text); });
  link.feed(fc.statusText("PreArm: GPS not healthy"));

  // Run
  withLog.poll(at(1.0));

  // Assert
  EXPECT_EQ(texts, (std::vector<std::string>{"PreArm: GPS not healthy"}));
}

TEST_F(MavlinkClientTest, FramesSplitAcrossReadsAndNoiseAreParsed)
{
  // Setup: garbage, then a heartbeat delivered 3 bytes per read, then a corrupted attitude
  link.maxChunk = 3;
  link.feed({0x00, 0xFD, 0x42, 0x13});
  link.feed(fc.heartbeat(models::kModeLoiter, false));
  std::vector<uint8_t> corrupted = fc.attitude(0.0F, 0.0F, 1.0F);
  corrupted[12] ^= 0xFF;
  link.feed(corrupted);
  link.feed(fc.attitude(0.0F, 0.0F, 2.0F));

  // Run
  int accepted = client.poll(at(1.0));

  // Assert
  EXPECT_EQ(accepted, 2);
  EXPECT_EQ(client.vehicle().customMode, models::kModeLoiter);
  EXPECT_FLOAT_EQ(client.vehicle().attitude.latest()->yaw, 2.0F);
}

TEST_F(MavlinkClientTest, SetpointCommandsBodyVelocityAndYawRateOnly)
{
  // Run
  client.poll(at(10.0));
  client.sendSetpoint({.vx = 1.25, .yawRate = -0.5}, at(10.5));

  // Assert
  std::vector<mavlink_message_t> sent = decodeFrames(link.sent);
  ASSERT_EQ(sent.size(), 1u);
  ASSERT_EQ(sent[0].msgid, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);
  EXPECT_EQ(sent[0].sysid, 1);
  EXPECT_EQ(sent[0].compid, 191);
  mavlink_set_position_target_local_ned_t target{};
  mavlink_msg_set_position_target_local_ned_decode(&sent[0], &target);
  EXPECT_EQ(target.target_system, 1);
  EXPECT_EQ(target.target_component, 1);
  EXPECT_EQ(target.coordinate_frame, MAV_FRAME_BODY_OFFSET_NED);
  EXPECT_EQ(target.type_mask, 0x05C7);
  EXPECT_EQ(target.time_boot_ms, 500u);
  EXPECT_FLOAT_EQ(target.vx, 1.25F);
  EXPECT_FLOAT_EQ(target.vy, 0.0F);
  EXPECT_FLOAT_EQ(target.vz, 0.0F);
  EXPECT_FLOAT_EQ(target.yaw_rate, -0.5F);
}

TEST_F(MavlinkClientTest, HeartbeatIdentifiesOnboardControllerOncePerSecond)
{
  // Run
  client.service(at(1.0));
  client.service(at(1.5));
  client.service(at(2.0));

  // Assert
  std::vector<mavlink_message_t> sent = decodeFrames(link.sent);
  ASSERT_EQ(sent.size(), 2u);
  mavlink_heartbeat_t heartbeat{};
  mavlink_msg_heartbeat_decode(&sent[0], &heartbeat);
  EXPECT_EQ(sent[0].msgid, MAVLINK_MSG_ID_HEARTBEAT);
  EXPECT_EQ(heartbeat.type, MAV_TYPE_ONBOARD_CONTROLLER);
  EXPECT_EQ(heartbeat.autopilot, MAV_AUTOPILOT_INVALID);
  EXPECT_EQ(sent[1].seq, static_cast<uint8_t>(sent[0].seq + 1));
}

TEST_F(MavlinkClientTest, StreamsAreRequestedUntilAttitudeArrives)
{
  // Setup: count SET_MESSAGE_INTERVAL commands sent since the last count
  auto streamRequests = [this] {
    int count = 0;
    for (const mavlink_message_t& message : decodeFrames(this->link.sent)) {
      if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG && mavlink_msg_command_long_get_command(&message) == MAV_CMD_SET_MESSAGE_INTERVAL) {
        ++count;
      }
    }
    this->link.sent.clear();
    return count;
  };

  // Run + Assert: nothing before the FC is heard
  client.service(at(0.0));
  EXPECT_EQ(streamRequests(), 0);

  // FC heard, no attitude: ATTITUDE and LOCAL_POSITION_NED requested, then again only after 2 s
  link.feed(fc.heartbeat(models::kModeLoiter, false));
  client.poll(at(0.5));
  client.service(at(0.5));
  EXPECT_EQ(streamRequests(), 2);
  client.service(at(1.5));
  EXPECT_EQ(streamRequests(), 0);
  client.service(at(2.5));
  EXPECT_EQ(streamRequests(), 2);

  // Both streams flowing: no more requests
  link.feed(fc.attitude(0.0F, 0.0F, 0.0F));
  link.feed(fc.localPosition(0.0F, 0.0F, -2.0F, 0.0F, 0.0F, 0.0F));
  client.poll(at(4.4));
  client.service(at(4.5));
  EXPECT_EQ(streamRequests(), 0);
}

TEST_F(MavlinkClientTest, StreamsAreRequestedUntilPositionArrives)
{
  // Setup: count SET_MESSAGE_INTERVAL commands sent since the last count
  auto streamRequests = [this] {
    int count = 0;
    for (const mavlink_message_t& message : decodeFrames(this->link.sent)) {
      if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG && mavlink_msg_command_long_get_command(&message) == MAV_CMD_SET_MESSAGE_INTERVAL) {
        ++count;
      }
    }
    this->link.sent.clear();
    return count;
  };

  // FC heard, and attitude flows from the start, but LOCAL_POSITION_NED never arrives.
  link.feed(fc.heartbeat(models::kModeLoiter, false));
  link.feed(fc.attitude(0.0F, 0.0F, 0.0F));
  client.poll(at(0.5));
  client.service(at(0.5));
  EXPECT_EQ(streamRequests(), 2);

  // Attitude kept fresh, position still missing: requests must keep coming every 2 s.
  link.feed(fc.attitude(0.0F, 0.0F, 0.0F));
  client.poll(at(1.4));
  client.service(at(1.5));
  EXPECT_EQ(streamRequests(), 0);  // not yet due

  link.feed(fc.attitude(0.0F, 0.0F, 0.0F));
  client.poll(at(2.4));
  client.service(at(2.5));
  EXPECT_EQ(streamRequests(), 2);
}

TEST_F(MavlinkClientTest, SendFailureIsReportedExactlyOnceUntilItRecovers)
{
  // Setup
  std::vector<std::string> texts;
  MavlinkClient withLog(link, MavlinkIds{}, [&texts](const std::string& text) { texts.push_back(text); });
  link.failSend = true;

  // Run: two sends while the link is down
  withLog.sendHeartbeat();
  withLog.sendHeartbeat();

  // Assert: exactly one report, not one per failed send
  EXPECT_EQ(texts, (std::vector<std::string>{"mavlink link send failed"}));

  // Run: the link recovers, then fails again
  link.failSend = false;
  withLog.sendHeartbeat();
  link.failSend = true;
  withLog.sendHeartbeat();

  // Assert: a second report, once, for the new failure
  EXPECT_EQ(texts, (std::vector<std::string>{"mavlink link send failed", "mavlink link send failed"}));
}

TEST_F(MavlinkClientTest, StreamRequestAsksForTwentyAndTenHertz)
{
  client.requestStreams();

  std::vector<mavlink_message_t> sent = decodeFrames(link.sent);
  ASSERT_EQ(sent.size(), 2u);
  mavlink_command_long_t attitude{};
  mavlink_command_long_t position{};
  mavlink_msg_command_long_decode(&sent[0], &attitude);
  mavlink_msg_command_long_decode(&sent[1], &position);
  EXPECT_EQ(attitude.target_system, 1);
  EXPECT_EQ(attitude.target_component, 1);
  EXPECT_EQ(attitude.param1, MAVLINK_MSG_ID_ATTITUDE);
  EXPECT_EQ(attitude.param2, 50000.0F);
  EXPECT_EQ(position.param1, MAVLINK_MSG_ID_LOCAL_POSITION_NED);
  EXPECT_EQ(position.param2, 100000.0F);
}
