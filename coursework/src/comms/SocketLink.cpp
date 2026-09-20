#include "comms/SocketLink.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include "PosixLink.h"
#include "interfaces/IByteLink.h"

namespace follow::comms {

SocketLink::SocketLink(int localPort, const std::string& remoteHost, int remotePort)
{
  this->fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (this->fd < 0) {
    throw std::runtime_error(detail::systemError("udp socket"));
  }
  int reuse = 1;
  ::setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  ::fcntl(this->fd, F_SETFL, ::fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);

  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_port = htons(static_cast<uint16_t>(localPort));
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(this->fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
    std::string message = detail::systemError("udp bind to port " + std::to_string(localPort));
    ::close(this->fd);
    throw std::runtime_error(message);
  }

  if (!remoteHost.empty()) {
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* resolved = nullptr;
    int rc = ::getaddrinfo(remoteHost.c_str(), std::to_string(remotePort).c_str(), &hints, &resolved);
    if (rc != 0 || resolved == nullptr) {
      ::close(this->fd);
      throw std::runtime_error("udp: cannot resolve '" + remoteHost + "': " + ::gai_strerror(rc));
    }
    std::memcpy(&this->peer, resolved->ai_addr, sizeof(this->peer));
    ::freeaddrinfo(resolved);
    this->peerKnown = true;
  }
}

SocketLink::~SocketLink()
{
  ::close(this->fd);
}

int SocketLink::localPort() const
{
  sockaddr_in local{};
  socklen_t length = sizeof(local);
  ::getsockname(this->fd, reinterpret_cast<sockaddr*>(&local), &length);
  return ntohs(local.sin_port);
}

int SocketLink::send(std::span<const uint8_t> bytes)
{
  if (!this->peerKnown) {
    return 0;
  }
  ssize_t sent = ::sendto(this->fd, bytes.data(), bytes.size(), 0, reinterpret_cast<const sockaddr*>(&this->peer), sizeof(this->peer));
  if (sent < 0) {
    return errno == EAGAIN || errno == EWOULDBLOCK ? 0 : -1;
  }
  return static_cast<int>(sent);
}

int SocketLink::receive(std::span<uint8_t> buffer)
{
  sockaddr_in from{};
  socklen_t fromLength = sizeof(from);
  ssize_t received = ::recvfrom(this->fd, buffer.data(), buffer.size(), 0, reinterpret_cast<sockaddr*>(&from), &fromLength);
  if (received < 0) {
    return errno == EAGAIN || errno == EWOULDBLOCK ? 0 : -1;
  }
  if (!this->peerKnown) {
    this->peer = from;
    this->peerKnown = true;
  }
  return static_cast<int>(received);
}

interfaces::IByteLink::WaitStatus SocketLink::waitReadable(std::chrono::milliseconds timeout)
{
  return detail::pollReadable(this->fd, timeout);
}

}  // namespace follow::comms
