#pragma once

#include "debug.h"
#include "gpio/IGpioController.h"

#include <string>

namespace gpio {

// Заглушка для збірок без libgpiod (macOS, CI, локальний тест на pty-парі).
// init() завжди успішний, а зміни ліній лише логуються - так решта пайплайну
// (UART, MAVLink, наведення) працює й перевіряється поза малиною.
class NullGpioController : public IGpioController {
public:
  bool init(const std::string &chip, unsigned startLine, unsigned dropLine) override
  {
    LOG("NullGpioController: chip=" << chip << " startLine=" << startLine << " dropLine=" << dropLine << " (no real GPIO)");
    return true;
  }

  void setStart(bool high) override { LOG("NullGpioController: START=" << (high ? 1 : 0)); }
  void setDrop(bool high) override { LOG("NullGpioController: DROP=" << (high ? 1 : 0)); }
};

}  // namespace gpio
