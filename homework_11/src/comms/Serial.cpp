#include "DronePhysics.h"
#include "models/drone_link.h"
#include "comms/Serial.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <any>
#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace dlink;

constexpr speed_t BAUD_RATE = B115200;

int Serial::open(const char* device)
{
  int fd = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    perror("open");
    return -1;
  }
  termios tio{};
  tcgetattr(fd, &tio);
  cfmakeraw(&tio);  // 8N1 format, no character handling

  // Set input and output speeds
  cfsetispeed(&tio, BAUD_RATE);
  cfsetospeed(&tio, BAUD_RATE);

  tio.c_cflag |= (CLOCAL | CREAD);  // non-blocking read
  tcsetattr(fd, TCSANOW, &tio);

  return fd;
}

std::any Serial::readFrame()
{
  Parser parser{};
  uint8_t buf[256];
  int n = read(this->fd, buf, sizeof(buf));
  uint8_t type, len, payload[260];
  for (int i = 0; i < n; i++)
    if (parser.feed(buf[i], type, payload, len)) {  // зібрався цілий кадр
      if (type == PKT_TELEMETRY) {
        Telemetry t;
        memcpy(&t, payload, sizeof(t));
        return t;
      }
      else if (type == PKT_TARGET) {
        TargetPos pos;
        memcpy(&pos, payload, sizeof(pos));
        return pos;
      }
      else if (type == PKT_AMMO) {
        AmmoCfg ammoCfg;
        memcpy(&ammoCfg, payload, sizeof(ammoCfg));
        return ammoCfg;
      }
    }
}