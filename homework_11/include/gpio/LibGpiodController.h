#pragma once
#if defined(USE_GPIOD)

#include "gpio/IGpioController.h"
#include <string>
#include <gpiod.h>

namespace gpio {

class LibGpiodController : public IGpioController {
private:
  gpiod_chip *chip{nullptr};
  gpiod_line *startLineHandle{nullptr};
  gpiod_line *dropLineHandle{nullptr};
  void release();

public:
  LibGpiodController() = default;
  ~LibGpiodController() override;

  bool init(const std::string &chip, unsigned startLine, unsigned dropLine) override;
  void setStart(bool high) override;
  void setDrop(bool high) override;
};

}  // namespace gpio
#endif
