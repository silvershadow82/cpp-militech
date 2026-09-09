#include "comms/SocketLink.h"
#include "debug.h"

#include <cstring>
#include <cerrno>
#include <iostream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>

namespace comms {

SocketLink::SocketLink(int port, const std::string &remoteHost, int remotePort)
  : port(port)
{
  this->socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (this->socket_fd < 0) {
    std::cerr << "socket error: " << strerror(errno) << std::endl;
    return;
  }

  int reuse = 1;
  setsockopt(this->socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  // Неблокуючий сокет: rx_poll() вичитує чергу до EAGAIN і повертає керування,
  // тому потік опитування завжди бачить stopRequested і коректно завершується.
  int flags = fcntl(this->socket_fd, F_GETFL, 0);
  if (flags < 0 || fcntl(this->socket_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    std::cerr << "fcntl O_NONBLOCK error: " << strerror(errno) << std::endl;
    close(this->socket_fd);
    this->socket_fd = -1;
    return;
  }

  this->addr.sin_family = AF_INET;
  this->addr.sin_port = htons(static_cast<uint16_t>(this->port));
  this->addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(this->socket_fd, reinterpret_cast<sockaddr *>(&this->addr), sizeof(this->addr)) != 0) {
    std::cerr << "bind error on port " << this->port << ": " << strerror(errno) << std::endl;
    close(this->socket_fd);
    this->socket_fd = -1;
    return;
  }

  if (!remoteHost.empty() && remotePort > 0) {
    // getaddrinfo, а не inet_pton: inet_pton розуміє лише числові адреси, тож
    // імена на кшталт host.docker.internal чи gcs.local тихо не працювали.
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    addrinfo *resolved = nullptr;
    const std::string service = std::to_string(remotePort);
    const int rc = getaddrinfo(remoteHost.c_str(), service.c_str(), &hints, &resolved);

    if (rc != 0 || resolved == nullptr) {
      std::cerr << "cannot resolve remote host '" << remoteHost << "': " << gai_strerror(rc) << std::endl;
    }
    else {
      std::memcpy(&this->peer_addr, resolved->ai_addr, sizeof(this->peer_addr));
      this->peer_known.store(true);
      freeaddrinfo(resolved);
      LOG("MAVLink remote resolved: " << remoteHost << " -> " << this->peerDescription());
    }
  }
}

SocketLink::~SocketLink()
{
  if (this->socket_fd >= 0) {
    close(this->socket_fd);
  }
}

std::string SocketLink::peerDescription() const
{
  if (!this->peer_known.load()) {
    return "<none>";
  }
  std::lock_guard<std::mutex> lock(this->peer_mtx);
  char ip[INET_ADDRSTRLEN]{};
  inet_ntop(AF_INET, &this->peer_addr.sin_addr, ip, sizeof(ip));
  return std::string(ip) + ":" + std::to_string(ntohs(this->peer_addr.sin_port));
}

int SocketLink::send(const uint8_t *buf, size_t len)
{
  if (this->socket_fd < 0 || !this->peer_known.load()) {
    return 0;
  }

  sockaddr_in dst{};
  {
    std::lock_guard<std::mutex> lock(this->peer_mtx);
    dst = this->peer_addr;
  }

  return static_cast<int>(sendto(this->socket_fd, buf, len, 0, reinterpret_cast<const sockaddr *>(&dst), sizeof(dst)));
}

int SocketLink::receive(uint8_t *buf, size_t len)
{
  if (this->socket_fd < 0) {
    return -1;
  }

  sockaddr_in from{};
  socklen_t fromLen = sizeof(from);
  ssize_t n = recvfrom(this->socket_fd, buf, len, 0, reinterpret_cast<sockaddr *>(&from), &fromLen);

  if (n > 0 && !this->peer_known.load()) {
    // Вивчаємо адресу GCS з першої датаграми і далі шлемо телеметрію туди.
    {
      std::lock_guard<std::mutex> lock(this->peer_mtx);
      this->peer_addr = from;
    }
    this->peer_known.store(true);
    LOG("MAVLink peer learned: " << this->peerDescription());
  }

  return static_cast<int>(n);
}
}  // namespace comms
