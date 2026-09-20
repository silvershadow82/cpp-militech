#pragma once

#include <netinet/in.h>
#include <termios.h>

#include <string>

#include "comms/ByteLink.h"

namespace follow::comms {

// UDP endpoint bound to localPort (0 = any free port). Sends to the given remote, or, without one,
// to the sender of the first datagram received (ArduPilot SITL's "udpclient" connects this way).
class UdpLink final : public ByteLink {
public:
  explicit UdpLink(int localPort, const std::string& remoteHost = {}, int remotePort = 0);
  ~UdpLink() override;
  UdpLink(const UdpLink&) = delete;
  UdpLink& operator=(const UdpLink&) = delete;

  int localPort() const;

  int send(std::span<const uint8_t> bytes) override;
  int receive(std::span<uint8_t> buffer) override;
  WaitStatus waitReadable(std::chrono::milliseconds timeout) override;

private:
  int fd{-1};
  sockaddr_in peer{};
  bool peerKnown{false};
};

// Serial port in raw 8N1, non-blocking, no flow control.
class UartLink final : public ByteLink {
public:
  UartLink(const std::string& device, int baud);
  ~UartLink() override;
  UartLink(const UartLink&) = delete;
  UartLink& operator=(const UartLink&) = delete;

  int send(std::span<const uint8_t> bytes) override;
  int receive(std::span<uint8_t> buffer) override;
  WaitStatus waitReadable(std::chrono::milliseconds timeout) override;

private:
  static speed_t speedFor(int baud);

  int fd{-1};
};

}  // namespace follow::comms
