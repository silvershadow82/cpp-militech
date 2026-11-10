#pragma once

#include <chrono>
#include <cstdint>
#include <span>

namespace follow::interfaces {

// Byte transport under MAVLink. Implementations are not thread-safe: one thread owns a link.
//
// No OpenCV here, and none in ICameraModel either: follow_core depends on both, and follow_core
// must keep building with FOLLOW_WITH_OPENCV=OFF.
class IByteLink {
public:
  // Outcome of waitReadable: whether the wait itself failed is distinguished from an idle timeout so
  // callers can back off and report a link failure instead of busy-spinning on a broken descriptor.
  enum class WaitStatus { Readable, Timeout, Error };

  virtual ~IByteLink() = default;

  // Bytes written; 0 if the link has nowhere to send yet (UDP before the peer is known) or the send buffer is full; -1 on error.
  virtual int send(std::span<const uint8_t> bytes) = 0;
  // Non-blocking read of what is available: bytes read, 0 if nothing, -1 on error.
  virtual int receive(std::span<uint8_t> buffer) = 0;
  // Waits up to timeout for incoming bytes: Readable if receive() has something to read, Timeout if
  // nothing arrived in time, Error if the wait itself failed (e.g. poll() returned -1).
  virtual WaitStatus waitReadable(std::chrono::milliseconds timeout) = 0;
};

}  // namespace follow::interfaces
