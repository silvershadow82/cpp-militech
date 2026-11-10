#pragma once

#include <memory>
#include <string>

#include "interfaces/IByteLink.h"

namespace follow::comms {

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
std::unique_ptr<interfaces::IByteLink> openLink(const LinkSpec& spec);

}  // namespace follow::comms
