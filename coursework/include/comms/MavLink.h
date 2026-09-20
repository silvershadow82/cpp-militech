#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "Types.h"
#include "interfaces/IByteLink.h"
#include "control/Core.h"

namespace follow::comms {

struct MavlinkIds {
  uint8_t sysid{1};     // ours: same system as the FC
  uint8_t compid{191};  // ours: MAV_COMP_ID_ONBOARD_COMPUTER
  uint8_t fcSysid{1};   // the autopilot we listen to and command
  uint8_t fcCompid{1};  // MAV_COMP_ID_AUTOPILOT1
};

// MAVLink 2 over an IByteLink: decodes the FC's telemetry into a VehicleState and encodes our
// heartbeat, stream requests and velocity setpoints. Not thread-safe: one I/O thread owns it.
class MavLink {
public:
  using StatusTextHandler = std::function<void(const std::string&)>;

  MavLink(interfaces::IByteLink& link, const MavlinkIds& ids, StatusTextHandler onStatusText = {});
  ~MavLink();
  MavLink(const MavLink&) = delete;
  MavLink& operator=(const MavLink&) = delete;

  // Reads and parses everything the link has. Messages from the FC are stamped with `now`.
  // Returns the number of FC messages accepted.
  int poll(models::TimePoint now);

  // Periodic duties: HEARTBEAT at 1 Hz, and while ATTITUDE or LOCAL_POSITION_NED is missing (never
  // received, or older than 1 s) after the FC has been heard, a stream request every 2 s.
  void service(models::TimePoint now);

  void sendHeartbeat();
  // SET_MESSAGE_INTERVAL for ATTITUDE at 20 Hz and LOCAL_POSITION_NED at 10 Hz.
  void requestStreams();
  // SET_POSITION_TARGET_LOCAL_NED in MAV_FRAME_BODY_OFFSET_NED using only vx and yaw_rate.
  void sendSetpoint(const models::VelocityCmd& command, models::TimePoint now);

  const control::VehicleState& vehicle() const { return this->state; }

private:
  struct Codec;  // MAVLink parser and sequence state, kept out of this header

  void handle(models::TimePoint now);
  void sendMessage();

  interfaces::IByteLink& link;
  MavlinkIds ids;
  StatusTextHandler onStatusText;
  std::unique_ptr<Codec> codec;
  control::VehicleState state{};
  std::optional<models::TimePoint> start{};
  std::optional<models::TimePoint> lastHeartbeatSent{};
  std::optional<models::TimePoint> lastStreamRequest{};
  bool sendFailing{false};  // reports through onStatusText once, on the transition into failure
};

}  // namespace follow::comms
