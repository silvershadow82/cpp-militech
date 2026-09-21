#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <common/mavlink.h>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

#include "interfaces/IByteLink.h"

namespace follow::test {

// In-memory link: tests append inbound bytes and inspect what was sent.
class FakeLink : public interfaces::IByteLink {
public:
  int send(std::span<const uint8_t> bytes) override
  {
    if (this->failSend) {
      return -1;
    }
    this->sent.emplace_back(bytes.begin(), bytes.end());
    return static_cast<int>(bytes.size());
  }

  int receive(std::span<uint8_t> buffer) override
  {
    size_t n = std::min({buffer.size(), this->inbound.size(), this->maxChunk});
    std::copy_n(this->inbound.begin(), n, buffer.begin());
    this->inbound.erase(this->inbound.begin(), this->inbound.begin() + static_cast<long>(n));
    return static_cast<int>(n);
  }

  WaitStatus waitReadable(std::chrono::milliseconds) override
  {
    if (this->failWaitReadable) {
      return WaitStatus::Error;
    }
    return this->inbound.empty() ? WaitStatus::Timeout : WaitStatus::Readable;
  }

  void feed(const std::vector<uint8_t>& bytes) { this->inbound.insert(this->inbound.end(), bytes.begin(), bytes.end()); }

  std::vector<uint8_t> inbound;
  std::vector<std::vector<uint8_t>> sent;
  size_t maxChunk{std::numeric_limits<size_t>::max()};
  bool failWaitReadable{false};  // makes waitReadable() report WaitStatus::Error, as a broken fd would
  bool failSend{false};          // makes send() return -1, as a dead link would
};

// Encodes messages the way a MAVLink peer with the given ids would.
class Peer {
public:
  Peer(uint8_t sysid, uint8_t compid)
    : sysid(sysid)
    , compid(compid)
  {
  }

  std::vector<uint8_t> heartbeat(uint32_t customMode, bool armed)
  {
    uint8_t baseMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0);
    mavlink_msg_heartbeat_pack_status(this->sysid,
                                      this->compid,
                                      &this->status,
                                      &this->message,
                                      MAV_TYPE_QUADROTOR,
                                      MAV_AUTOPILOT_ARDUPILOTMEGA,
                                      baseMode,
                                      customMode,
                                      MAV_STATE_ACTIVE);
    return this->bytes();
  }

  std::vector<uint8_t> attitude(float roll, float pitch, float yaw)
  {
    mavlink_msg_attitude_pack_status(this->sysid, this->compid, &this->status, &this->message, 1000, roll, pitch, yaw, 0.0F, 0.0F, 0.0F);
    return this->bytes();
  }

  std::vector<uint8_t> localPosition(float x, float y, float z, float vx, float vy, float vz)
  {
    mavlink_msg_local_position_ned_pack_status(this->sysid, this->compid, &this->status, &this->message, 1000, x, y, z, vx, vy, vz);
    return this->bytes();
  }

  std::vector<uint8_t> statusText(const char* text)
  {
    mavlink_msg_statustext_pack_status(this->sysid, this->compid, &this->status, &this->message, MAV_SEVERITY_INFO, text, 0, 0);
    return this->bytes();
  }

private:
  std::vector<uint8_t> bytes() const
  {
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    uint16_t length = mavlink_msg_to_send_buffer(buffer.data(), &this->message);
    return {buffer.begin(), buffer.begin() + length};
  }

  uint8_t sysid;
  uint8_t compid;
  mavlink_status_t status{};
  mavlink_message_t message{};
};

// Decodes every complete MAVLink message in a sequence of sent frames.
inline std::vector<mavlink_message_t> decodeFrames(const std::vector<std::vector<uint8_t>>& frames)
{
  std::vector<mavlink_message_t> messages;
  mavlink_message_t buffer{};
  mavlink_status_t status{};
  for (const std::vector<uint8_t>& frame : frames) {
    for (uint8_t c : frame) {
      mavlink_message_t message{};
      mavlink_status_t messageStatus{};
      if (mavlink_frame_char_buffer(&buffer, &status, c, &message, &messageStatus) == MAVLINK_FRAMING_OK) {
        messages.push_back(message);
      }
    }
  }
  return messages;
}

}  // namespace follow::test
