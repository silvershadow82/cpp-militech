#pragma once

#include <atomic>
#include <cstdint>

#include "Types.h"
#include "interfaces/IByteLink.h"
#include "comms/MavLink.h"
#include "util/Channels.h"

namespace follow::comms {

// The MAVLink I/O thread: owns the link and the client, publishes the vehicle state and sends the
// setpoints the control loop writes.
class MavlinkIo {
public:
  MavlinkIo(interfaces::IByteLink& link,
            const comms::MavlinkIds& ids,
            util::Channels& channels,
            comms::MavLink::StatusTextHandler onStatusText = {});

  // One pass: parse what arrived and publish the vehicle state if the FC said anything, do the
  // heartbeat and stream-request duties, and send the setpoint if a new one was written.
  void iterate(models::TimePoint now);

  // iterate() whenever bytes arrive and at least every 5 ms, until `stop` is set. If waitReadable
  // itself fails (a broken file descriptor), backs off with a short sleep instead of busy-spinning,
  // and reports the failure once through onStatusText.
  void run(const std::atomic<bool>& stop);

  // Sequence number of the last setpoint handed to the link, so a caller that must know its
  // fail-safe setpoint really went out can wait for it before it stops this thread.
  uint64_t sentSetpointSequence() const { return this->sentSetpoint; }

private:
  interfaces::IByteLink& link;
  comms::MavLink client;
  util::Channels& channels;
  comms::MavLink::StatusTextHandler onStatusText;
  std::atomic<uint64_t> sentSetpoint{0};  // sequence number of the last setpoint sent
  bool waitFailing{false};                // reports through onStatusText once, on the transition into failure
};

}  // namespace follow::comms
