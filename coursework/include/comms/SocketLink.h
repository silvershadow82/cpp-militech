#pragma once

#include <netinet/in.h>

#include <string>

#include "interfaces/IByteLink.h"

namespace follow::comms {

// UDP endpoint bound to localPort (0 = any free port). Sends to the given remote, or, without one,
// to the sender of the first datagram received (ArduPilot SITL's "udpclient" connects this way).
class SocketLink final : public interfaces::IByteLink {
public:
  explicit SocketLink(int localPort, const std::string& remoteHost = {}, int remotePort = 0);
  ~SocketLink() override;
  SocketLink(const SocketLink&) = delete;
  SocketLink& operator=(const SocketLink&) = delete;

  int localPort() const;

  int send(std::span<const uint8_t> bytes) override;
  int receive(std::span<uint8_t> buffer) override;
  WaitStatus waitReadable(std::chrono::milliseconds timeout) override;

private:
  int fd{-1};
  sockaddr_in peer{};
  bool peerKnown{false};
};

}  // namespace follow::comms
