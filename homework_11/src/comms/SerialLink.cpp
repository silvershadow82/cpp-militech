#include "comms/SerialLink.h"
#include "models/drone_link.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>

namespace comms
{

  namespace
  {
    constexpr speed_t BAUD_RATE = B115200;
    // Захист від необмеженого росту txpending, якщо приймач ніколи не читає:
    // найновіше CONTROL найважливіше, тож при переповненні відкидаємо старе.
    constexpr size_t TX_BACKLOG_LIMIT = 4096;

    // Чи це один із типів протоколу drone_link
    bool isKnownType(uint8_t type)
    {
      switch (type)
      {
      case dlink::PKT_TELEMETRY:
      case dlink::PKT_TARGET:
      case dlink::PKT_AMMO:
      case dlink::PKT_RESULT:
      case dlink::PKT_CONTROL:
      case dlink::PKT_CONFIG:
        return true;
      default:
        return false;
      }
    }
  }

  SerialLink::~SerialLink()
  {
    if (this->fd >= 0)
    {
      close(this->fd);
    }
  }

  bool SerialLink::open(const char *dev)
  {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
      perror("SerialLink::open");
      return false;
    }

    termios tio{};
    if (tcgetattr(fd, &tio) == 0)
    {                               // справжній tty — налаштовуємо лінію
      cfmakerraw(&tio);             // 8N1, без обробки символів
      cfsetispeed(&tio, BAUD_RATE); // Однакова швидкість
      cfsetospeed(&tio, BAUD_RATE);
      tio.c_cflag |= (CLOCAL | CREAD);
      tcsetattr(fd, TCSANOW, &tio);
    }

    if (this->fd >= 0)
    {
      close(this->fd);
    }

    this->fd = fd;
    return true;
  }

  std::optional<Frame> SerialLink::drainBuffered()
  {
    uint8_t type = 0, len = 0;
    uint8_t payload[260];
    while (this->rxpos < this->rxlen)
    {
      if (this->parser.feed(this->rxbuf[this->rxpos++], type, payload, len))
      {
        if (!isKnownType(type))
          continue; // невідомий тип — ігноруємо, читаємо далі
        Frame frame;
        frame.ok = true;
        frame.type = static_cast<dlink::PacketType>(type);
        frame.len = len;
        std::memcpy(frame.payload.data(), payload, len);
        return frame;
      }
    }
    return std::nullopt;
  }

  Frame SerialLink::readFrame()
  {
    if (this->fd < 0)
    {
      return Frame{};
    }

    // 1) Спершу добираємо байти, що лишилися з попереднього read()
    if (auto frame = this->drainBuffered() && frame.ok)
      return frame;

    // 2) Буфер порожній — читаємо доступні байти один раз.
    ssize_t n = read(this->fd, this->rxbuf, sizeof(this->rxbuf));
    if (n <= 0)
    {
      this->rxlen = 0;
      this->rxpos = 0;
      return Frame{};
    }

    this->rxlen = static_cast<int>(n);
    this->rxpos = 0;

    return this->drainBuffered();
  }

  void SerialLink::writePending()
  {
    if (this->fd < 0)
    {
      return;
    }

    while (!this->txpending.empty())
    {
      ssize_t bytesWritten = write(this->fd, this->txpending.data(), this->txpending.size());

      if (bytesWritten > 0)
      {
        this->txpending.erase(this->txpending.begin(), this->txpending.begin() + bytesWritten);
      }
      else
      {
        break;
      }
    }
  }

  void SerialLink::sendControl(float accel, float turnRate)
  {
    dlink::Control control{accel, turnRate};
    uint8_t out[64];
    size_t n = dlink::encode(dlink::PKT_CONTROL, &control, sizeof(control), out);

    // Захист від необмеженого росту txpending, якщо приймач ніколи не читає
    if (this->txpending.size() > TX_PENDING_LIMIT)
    {
      this->txpending.clear();
    }

    this->txpending.insert(this->txpending.end(), out, out + n);

    this->writePending();
  }

}
