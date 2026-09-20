#pragma once

#include <poll.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <string>

#include "interfaces/IByteLink.h"

// Shared between SocketLink.cpp and SerialLink.cpp: both wrap a POSIX file descriptor.
namespace follow::comms::detail {

inline interfaces::IByteLink::WaitStatus pollReadable(int fd, std::chrono::milliseconds timeout)
{
  pollfd entry{.fd = fd, .events = POLLIN, .revents = 0};
  int rc = ::poll(&entry, 1, static_cast<int>(timeout.count()));
  if (rc < 0) {
    return interfaces::IByteLink::WaitStatus::Error;
  }
  if (rc > 0 && (entry.revents & POLLIN) != 0) {
    return interfaces::IByteLink::WaitStatus::Readable;
  }
  return interfaces::IByteLink::WaitStatus::Timeout;
}

inline std::string systemError(const std::string& what)
{
  return what + ": " + std::strerror(errno);
}

}  // namespace follow::comms::detail
