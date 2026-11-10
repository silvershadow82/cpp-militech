#pragma once

#include <termios.h>

#include <string>

#include "interfaces/IByteLink.h"

namespace follow::comms {

// Serial port in raw 8N1, non-blocking, no flow control.
class SerialLink final : public interfaces::IByteLink {
public:
  SerialLink(const std::string& device, int baud);
  ~SerialLink() override;
  SerialLink(const SerialLink&) = delete;
  SerialLink& operator=(const SerialLink&) = delete;

  int send(std::span<const uint8_t> bytes) override;
  int receive(std::span<uint8_t> buffer) override;
  WaitStatus waitReadable(std::chrono::milliseconds timeout) override;

private:
  static speed_t speedFor(int baud);

  int fd{-1};
};

}  // namespace follow::comms
