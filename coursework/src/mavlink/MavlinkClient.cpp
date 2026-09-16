#include "follow/mavlink/MavlinkClient.h"

#include <array>
#include <chrono>
#include <common/mavlink.h>
#include <cstring>
#include <utility>

namespace follow::mavlink {

namespace {

constexpr auto kHeartbeatPeriod = std::chrono::seconds{1};
constexpr auto kStreamRequestPeriod = std::chrono::seconds{2};
constexpr auto kAttitudeMissingAfter = std::chrono::seconds{1};
constexpr float kAttitudeIntervalUs = 50000.0F;        // 20 Hz
constexpr float kLocalPositionIntervalUs = 100000.0F;  // 10 Hz
// Ignore position (bits 0-2), acceleration (6-8) and yaw (10): use velocity and yaw_rate.
constexpr uint16_t kVelocityYawRateMask = 0x05C7;

}  // namespace

struct MavlinkClient::Codec {
  mavlink_message_t rxBuffer{};
  mavlink_status_t rxStatus{};
  mavlink_message_t received{};
  mavlink_status_t receivedStatus{};
  mavlink_status_t txStatus{};  // our outgoing sequence number
  mavlink_message_t outgoing{};
  std::optional<core::AttitudeSample> pendingAttitude{};
};

MavlinkClient::MavlinkClient(ByteLink& link, const MavlinkIds& ids, StatusTextHandler onStatusText)
  : link(link)
  , ids(ids)
  , onStatusText(std::move(onStatusText))
  , codec(std::make_unique<Codec>())
{
}

MavlinkClient::~MavlinkClient() = default;

int MavlinkClient::poll(core::TimePoint now)
{
  if (!this->start) {
    this->start = now;
  }
  Codec& k = *this->codec;
  std::array<uint8_t, 512> buffer{};
  int accepted = 0;
  for (int n = this->link.receive(buffer); n > 0; n = this->link.receive(buffer)) {
    for (int i = 0; i < n; ++i) {
      uint8_t c = buffer[static_cast<size_t>(i)];
      uint8_t result = mavlink_frame_char_buffer(&k.rxBuffer, &k.rxStatus, c, &k.received, &k.receivedStatus);
      if (result == MAVLINK_FRAMING_BAD_CRC || result == MAVLINK_FRAMING_BAD_SIGNATURE) {
        // Same recovery as mavlink_parse_char. Messages outside the common dialect also end here,
        // because their CRC seed is unknown.
        k.rxStatus.msg_received = MAVLINK_FRAMING_INCOMPLETE;
        k.rxStatus.parse_state = MAVLINK_PARSE_STATE_IDLE;
        if (c == MAVLINK_STX) {
          k.rxStatus.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
          k.rxBuffer.len = 0;
          mavlink_start_checksum(&k.rxBuffer);
        }
      }
      else if (result == MAVLINK_FRAMING_OK && k.received.sysid == this->ids.fcSysid && k.received.compid == this->ids.fcCompid) {
        this->handle(now);
        ++accepted;
      }
    }
  }
  // Every sample of one poll shares `now`, and the history ignores samples that are not newer,
  // so only the newest attitude of the batch is stored.
  if (k.pendingAttitude) {
    this->state.attitude.push(*k.pendingAttitude);
    k.pendingAttitude.reset();
  }
  return accepted;
}

void MavlinkClient::handle(core::TimePoint now)
{
  const mavlink_message_t& message = this->codec->received;
  switch (message.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      mavlink_heartbeat_t heartbeat{};
      mavlink_msg_heartbeat_decode(&message, &heartbeat);
      this->state.lastHeartbeat = now;
      this->state.customMode = heartbeat.custom_mode;
      this->state.armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
      break;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
      mavlink_attitude_t attitude{};
      mavlink_msg_attitude_decode(&message, &attitude);
      this->codec->pendingAttitude = core::AttitudeSample{.t = now, .roll = attitude.roll, .pitch = attitude.pitch, .yaw = attitude.yaw};
      break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
      mavlink_local_position_ned_t local{};
      mavlink_msg_local_position_ned_decode(&message, &local);
      this->state.position =
        core::LocalPositionNed{.t = now, .position = {local.x, local.y, local.z}, .velocity = {local.vx, local.vy, local.vz}};
      break;
    }
    case MAVLINK_MSG_ID_STATUSTEXT: {
      mavlink_statustext_t status{};
      mavlink_msg_statustext_decode(&message, &status);
      if (this->onStatusText) {
        this->onStatusText(std::string(status.text, strnlen(status.text, sizeof(status.text))));
      }
      break;
    }
    default:
      break;
  }
}

void MavlinkClient::service(core::TimePoint now)
{
  if (!this->start) {
    this->start = now;
  }
  if (!this->lastHeartbeatSent || now - *this->lastHeartbeatSent >= kHeartbeatPeriod) {
    this->sendHeartbeat();
    this->lastHeartbeatSent = now;
  }
  std::optional<core::AttitudeSample> latest = this->state.attitude.latest();
  bool attitudeMissing = !latest || now - latest->t > kAttitudeMissingAfter;
  bool requestDue = !this->lastStreamRequest || now - *this->lastStreamRequest >= kStreamRequestPeriod;
  if (this->state.lastHeartbeat && attitudeMissing && requestDue) {
    this->requestStreams();
    this->lastStreamRequest = now;
  }
}

void MavlinkClient::sendHeartbeat()
{
  Codec& k = *this->codec;
  mavlink_msg_heartbeat_pack_status(this->ids.sysid,
                                    this->ids.compid,
                                    &k.txStatus,
                                    &k.outgoing,
                                    MAV_TYPE_ONBOARD_CONTROLLER,
                                    MAV_AUTOPILOT_INVALID,
                                    0,
                                    0,
                                    MAV_STATE_ACTIVE);
  this->sendMessage();
}

void MavlinkClient::requestStreams()
{
  Codec& k = *this->codec;
  for (auto [messageId, intervalUs] :
       {std::pair{MAVLINK_MSG_ID_ATTITUDE, kAttitudeIntervalUs}, std::pair{MAVLINK_MSG_ID_LOCAL_POSITION_NED, kLocalPositionIntervalUs}}) {
    mavlink_msg_command_long_pack_status(this->ids.sysid,
                                         this->ids.compid,
                                         &k.txStatus,
                                         &k.outgoing,
                                         this->ids.fcSysid,
                                         this->ids.fcCompid,
                                         MAV_CMD_SET_MESSAGE_INTERVAL,
                                         0,
                                         static_cast<float>(messageId),
                                         intervalUs,
                                         0.0F,
                                         0.0F,
                                         0.0F,
                                         0.0F,
                                         0.0F);
    this->sendMessage();
  }
}

void MavlinkClient::sendSetpoint(const core::VelocityCmd& command, core::TimePoint now)
{
  if (!this->start) {
    this->start = now;
  }
  Codec& k = *this->codec;
  auto timeBootMs = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now - *this->start).count());
  mavlink_msg_set_position_target_local_ned_pack_status(this->ids.sysid,
                                                        this->ids.compid,
                                                        &k.txStatus,
                                                        &k.outgoing,
                                                        timeBootMs,
                                                        this->ids.fcSysid,
                                                        this->ids.fcCompid,
                                                        MAV_FRAME_BODY_OFFSET_NED,
                                                        kVelocityYawRateMask,
                                                        0.0F,
                                                        0.0F,
                                                        0.0F,
                                                        static_cast<float>(command.vx),
                                                        0.0F,
                                                        0.0F,
                                                        0.0F,
                                                        0.0F,
                                                        0.0F,
                                                        0.0F,
                                                        static_cast<float>(command.yawRate));
  this->sendMessage();
}

void MavlinkClient::sendMessage()
{
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
  uint16_t length = mavlink_msg_to_send_buffer(bytes.data(), &this->codec->outgoing);
  this->link.send(std::span<const uint8_t>(bytes.data(), length));
}

}  // namespace follow::mavlink
