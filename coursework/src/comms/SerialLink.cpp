#include "comms/SerialLink.h"
#include "comms/PosixLink.h"
#include "interfaces/IByteLink.h"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <stdexcept>
#include <string>

namespace follow::comms {

SerialLink::SerialLink(const std::string &device, int baud)
{
  this->fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (this->fd < 0) {
    throw std::runtime_error(detail::systemError("uart open " + device));
  }
  termios tio{};
  if (::tcgetattr(this->fd, &tio) != 0) {
    std::string message = detail::systemError("uart " + device + " is not a terminal");
    ::close(this->fd);
    throw std::runtime_error(message);
  }
  ::cfmakeraw(&tio);  // 8N1, no echo, no line editing
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~CRTSCTS;
  if (::cfsetspeed(&tio, speedFor(baud)) != 0 || ::tcsetattr(this->fd, TCSANOW, &tio) != 0) {
    std::string message = detail::systemError("uart " + device + " at " + std::to_string(baud) + " baud");
    ::close(this->fd);
    throw std::runtime_error(message);
  }
}

SerialLink::~SerialLink()
{
  ::close(this->fd);
}

speed_t SerialLink::speedFor(int baud)
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

int SerialLink::send(std::span<const uint8_t> bytes)
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

int SerialLink::receive(std::span<uint8_t> buffer)
{
  ssize_t received = ::read(this->fd, buffer.data(), buffer.size());
  if (received < 0) {
    return errno == EAGAIN || errno == EWOULDBLOCK ? 0 : -1;
  }
  return static_cast<int>(received);
}

interfaces::IByteLink::WaitStatus SerialLink::waitReadable(std::chrono::milliseconds timeout)
{
  return detail::pollReadable(this->fd, timeout);
}

}  // namespace follow::comms
