#include "follow/runtime/MavlinkIo.h"

#include <chrono>
#include <utility>

namespace follow::runtime {

MavlinkIo::MavlinkIo(mavlink::ByteLink& link,
                     const mavlink::MavlinkIds& ids,
                     Channels& channels,
                     mavlink::MavlinkClient::StatusTextHandler onStatusText)
  : link(link)
  , client(link, ids, std::move(onStatusText))
  , channels(channels)
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
    this->link.waitReadable(std::chrono::milliseconds{5});
    this->iterate(core::Clock::now());
  }
}

}  // namespace follow::runtime
