#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace follow::mavlink {

// Byte transport under MAVLink. Implementations are not thread-safe: one thread owns a link.
class ByteLink {
public:
  virtual ~ByteLink() = default;

  // Bytes written; 0 if the link has nowhere to send yet (UDP before the peer is known) or the send buffer is full; -1 on error.
  virtual int send(std::span<const uint8_t> bytes) = 0;
  // Non-blocking read of what is available: bytes read, 0 if nothing, -1 on error.
  virtual int receive(std::span<uint8_t> buffer) = 0;
  // Waits up to timeout for incoming bytes; true if receive() has something to read.
  virtual bool waitReadable(std::chrono::milliseconds timeout) = 0;
};

struct LinkSpec {
  enum class Kind { Udp, Uart };
  Kind kind{Kind::Udp};
  int localPort{0};        // udp
  std::string remoteHost;  // udp; empty = reply to whoever sent the first datagram
  int remotePort{0};       // udp
  std::string device;      // uart
  int baud{0};             // uart
};

// "udp:LOCAL_PORT", "udp:LOCAL_PORT:REMOTE_HOST:REMOTE_PORT" or "uart:DEVICE:BAUD".
// Throws std::invalid_argument naming the problem.
LinkSpec parseLinkSpec(const std::string& text);

// Opens the link; throws std::runtime_error if the socket or device cannot be opened.
std::unique_ptr<ByteLink> openLink(const LinkSpec& spec);

}  // namespace follow::mavlink
