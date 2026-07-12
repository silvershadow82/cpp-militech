#pragma once

#include "comms/Frame.h"
#include "models/drone_link.h"
#include <cstdint>
#include <vector>

namespace comms {

class SerialLink {
public:
  SerialLink() = default;
  ~SerialLink();

  SerialLink(const SerialLink &) = delete;
  SerialLink &operator=(const SerialLink &) = delete;

  // Відкрити UART-пристрій: 115200 8N1, raw, O_NONBLOCK. false при помилці.
  bool open(const char *dev);

  // Прочитати один кадр протоколу drone_link
  Frame readFrame();

  // Спакувати PKT_CONTROL через dlink::encode() і записати
  void sendControl(float accel, float turnRate);

private:
  // Згодувати парсеру байти, що лишилися в rxbuf; повернути кадр, якщо зібрався.
  Frame drainBuffered();
  // Дописати незаписаний txpending
  void writePending();

  int fd{-1};
  dlink::Parser parser{};
  uint8_t rxbuf[256]{};              // байти, зчитані, але ще не згодовані парсеру
  int rxlen{0};                      // валідних байтів у rxbuf
  int rxpos{0};                      // позиція усередині rxbuf
  std::vector<uint8_t> txpending{};  // незаписані байти CONTROL
};

}  // namespace comms
