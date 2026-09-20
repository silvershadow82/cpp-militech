#include "comms/MavlinkIo.h"

#include <chrono>
#include <thread>
#include <utility>

namespace follow::comms {

namespace {

constexpr auto kWaitFailureBackoff = std::chrono::milliseconds{50};

}  // namespace

MavlinkIo::MavlinkIo(comms::ByteLink& link,
                     const comms::MavlinkIds& ids,
                     util::Channels& channels,
                     comms::MavlinkClient::StatusTextHandler onStatusText)
  : link(link)
  , client(link, ids, onStatusText)
  , channels(channels)
  , onStatusText(std::move(onStatusText))
{
}

void MavlinkIo::iterate(models::TimePoint now)
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
    comms::ByteLink::WaitStatus status = this->link.waitReadable(std::chrono::milliseconds{5});
    if (status == comms::ByteLink::WaitStatus::Error) {
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
    this->iterate(models::Clock::now());
  }
}

}  // namespace follow::comms
