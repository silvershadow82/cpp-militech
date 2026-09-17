#include "follow/runtime/MavlinkIo.h"

#include <chrono>
#include <thread>
#include <utility>

namespace follow::runtime {

namespace {

constexpr auto kWaitFailureBackoff = std::chrono::milliseconds{50};

}  // namespace

MavlinkIo::MavlinkIo(mavlink::ByteLink& link,
                     const mavlink::MavlinkIds& ids,
                     Channels& channels,
                     mavlink::MavlinkClient::StatusTextHandler onStatusText)
  : link(link)
  , client(link, ids, onStatusText)
  , channels(channels)
  , onStatusText(std::move(onStatusText))
{
}

void MavlinkIo::iterate(core::TimePoint now)
{
  if (this->client.poll(now) > 0) {
    this->channels.vehicle.write(this->client.vehicle(), now);
  }
  this->client.service(now);
  if (auto setpoint = this->channels.setpoint.read(); setpoint && setpoint->sequence != this->sentSetpoint) {
    this->client.sendSetpoint(setpoint->value, now);
    this->sentSetpoint = setpoint->sequence;
  }
}

void MavlinkIo::run(const std::atomic<bool>& stop)
{
  while (!stop) {
    mavlink::ByteLink::WaitStatus status = this->link.waitReadable(std::chrono::milliseconds{5});
    if (status == mavlink::ByteLink::WaitStatus::Error) {
      // Report only on the transition into the failing state, so a broken fd does not spam the log.
      if (!this->waitFailing) {
        this->waitFailing = true;
        if (this->onStatusText) {
          this->onStatusText("mavlink link wait failed");
        }
      }
      // Back off instead of busy-spinning on a descriptor that keeps failing poll().
      std::this_thread::sleep_for(kWaitFailureBackoff);
    }
    else {
      this->waitFailing = false;
    }
    this->iterate(core::Clock::now());
  }
}

}  // namespace follow::runtime
