#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "follow/mavlink/ByteLink.h"
#include "follow/mavlink/Links.h"

namespace follow::mavlink {

namespace {

std::vector<std::string> split(const std::string& text, char separator)
{
  std::vector<std::string> parts;
  size_t start = 0;
  while (true) {
    size_t end = text.find(separator, start);
    parts.push_back(text.substr(start, end - start));
    if (end == std::string::npos) {
      return parts;
    }
    start = end + 1;
  }
}

// Whole-string decimal number in [minimum, maximum].
int parseNumber(const std::string& text, int minimum, int maximum, const std::string& what, const std::string& spec)
{
  size_t used = 0;
  long value = -1;
  try {
    value = std::stol(text, &used);
  }
  catch (const std::exception&) {
    used = 0;
  }
  if (text.empty() || used != text.size() || value < minimum || value > maximum) {
    throw std::invalid_argument("bad " + what + " '" + text + "' in link '" + spec + "'");
  }
  return static_cast<int>(value);
}

bool pollReadable(int fd, std::chrono::milliseconds timeout)
{
  pollfd entry{.fd = fd, .events = POLLIN, .revents = 0};
  return ::poll(&entry, 1, static_cast<int>(timeout.count())) > 0 && (entry.revents & POLLIN) != 0;
}

std::string systemError(const std::string& what)
{
  return what + ": " + std::strerror(errno);
}

}  // namespace

LinkSpec parseLinkSpec(const std::string& text)
{
  std::vector<std::string> parts = split(text, ':');
  LinkSpec spec;
  if (parts[0] == "udp" && (parts.size() == 2 || parts.size() == 4)) {
    spec.kind = LinkSpec::Kind::Udp;
    spec.localPort = parseNumber(parts[1], 0, 65535, "port", text);
    if (parts.size() == 4) {
      spec.remoteHost = parts[2];
      spec.remotePort = parseNumber(parts[3], 1, 65535, "port", text);
    }
    return spec;
  }
  if (parts[0] == "uart" && parts.size() == 3 && !parts[1].empty()) {
    spec.kind = LinkSpec::Kind::Uart;
    spec.device = parts[1];
    spec.baud = parseNumber(parts[2], 1, 4000000, "baud rate", text);
    return spec;
  }
  throw std::invalid_argument("link '" + text + "': expected udp:PORT, udp:PORT:HOST:PORT or uart:DEVICE:BAUD");
}

std::unique_ptr<ByteLink> openLink(const LinkSpec& spec)
{
  if (spec.kind == LinkSpec::Kind::Udp) {
    return std::make_unique<UdpLink>(spec.localPort, spec.remoteHost, spec.remotePort);
  }
  return std::make_unique<UartLink>(spec.device, spec.baud);
}

// --- UdpLink ------------------------------------------------------------------

UdpLink::UdpLink(int localPort, const std::string& remoteHost, int remotePort)
{
  this->fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (this->fd < 0) {
    throw std::runtime_error(systemError("udp socket"));
  }
  int reuse = 1;
  ::setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  ::fcntl(this->fd, F_SETFL, ::fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);

  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_port = htons(static_cast<uint16_t>(localPort));
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(this->fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
    std::string message = systemError("udp bind to port " + std::to_string(localPort));
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

UdpLink::~UdpLink()
{
  ::close(this->fd);
}

int UdpLink::localPort() const
{
  sockaddr_in local{};
  socklen_t length = sizeof(local);
  ::getsockname(this->fd, reinterpret_cast<sockaddr*>(&local), &length);
  return ntohs(local.sin_port);
}

int UdpLink::send(std::span<const uint8_t> bytes)
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

int UdpLink::receive(std::span<uint8_t> buffer)
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

bool UdpLink::waitReadable(std::chrono::milliseconds timeout)
{
  return pollReadable(this->fd, timeout);
}

// --- UartLink -----------------------------------------------------------------

UartLink::UartLink(const std::string& device, int baud)
{
  this->fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (this->fd < 0) {
    throw std::runtime_error(systemError("uart open " + device));
  }
  termios tio{};
  if (::tcgetattr(this->fd, &tio) != 0) {
    std::string message = systemError("uart " + device + " is not a terminal");
    ::close(this->fd);
    throw std::runtime_error(message);
  }
  ::cfmakeraw(&tio);  // 8N1, no echo, no line editing
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~CRTSCTS;
  if (::cfsetspeed(&tio, speedFor(baud)) != 0 || ::tcsetattr(this->fd, TCSANOW, &tio) != 0) {
    std::string message = systemError("uart " + device + " at " + std::to_string(baud) + " baud");
    ::close(this->fd);
    throw std::runtime_error(message);
  }
}

UartLink::~UartLink()
{
  ::close(this->fd);
}

speed_t UartLink::speedFor(int baud)
{
#ifdef __linux__
  switch (baud) {
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      throw std::runtime_error("uart: unsupported baud rate " + std::to_string(baud));
  }
#else
  return static_cast<speed_t>(baud);  // BSD and macOS take the numeric rate
#endif
}

int UartLink::send(std::span<const uint8_t> bytes)
{
  size_t total = 0;
  while (total < bytes.size()) {
    ssize_t written = ::write(this->fd, bytes.data() + total, bytes.size() - total);
    if (written > 0) {
      total += static_cast<size_t>(written);
    }
    else if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd entry{.fd = this->fd, .events = POLLOUT, .revents = 0};
      if (::poll(&entry, 1, 5) <= 0) {
        break;  // output buffer stays full: drop the rest rather than stall the I/O thread
      }
    }
    else {
      return -1;
    }
  }
  return static_cast<int>(total);
}

int UartLink::receive(std::span<uint8_t> buffer)
{
  ssize_t received = ::read(this->fd, buffer.data(), buffer.size());
  if (received < 0) {
    return errno == EAGAIN || errno == EWOULDBLOCK ? 0 : -1;
  }
  return static_cast<int>(received);
}

bool UartLink::waitReadable(std::chrono::milliseconds timeout)
{
  return pollReadable(this->fd, timeout);
}

}  // namespace follow::mavlink
