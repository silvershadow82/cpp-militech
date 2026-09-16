#pragma once

#include <atomic>
#include <cstdint>

#include "follow/core/Types.h"
#include "follow/mavlink/ByteLink.h"
#include "follow/mavlink/MavlinkClient.h"
#include "follow/runtime/Channels.h"

namespace follow::runtime {

// The MAVLink I/O thread: owns the link and the client, publishes the vehicle state and sends the
// setpoints the control loop writes.
class MavlinkIo {
public:
  MavlinkIo(mavlink::ByteLink& link,
            const mavlink::MavlinkIds& ids,
            Channels& channels,
            mavlink::MavlinkClient::StatusTextHandler onStatusText = {});

  // One pass: parse what arrived and publish the vehicle state if the FC said anything, do the
  // heartbeat and stream-request duties, and send the setpoint if a new one was written.
  void iterate(core::TimePoint now);

  // iterate() whenever bytes arrive and at least every 5 ms, until `stop` is set.
  void run(const std::atomic<bool>& stop);

private:
  mavlink::ByteLink& link;
  mavlink::MavlinkClient client;
  Channels& channels;
  uint64_t sentSetpoint{0};  // sequence number of the last setpoint sent
};

}  // namespace follow::runtime
