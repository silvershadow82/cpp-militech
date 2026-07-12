#if defined(USE_GPIOD)

#include "gpio/IGpioController.h"
#include "debug.h"

#include <gpiod.h>

#include <memory>
#include <string>

namespace gpio {

class LibGpiodController : public IGpioController {
public:
  LibGpiodController() = default;
  ~LibGpiodController() override { this->release(); }

  bool init(const std::string &chip, int startLine, int dropLine) override
  {
    this->release();

    this->chip = gpiod_chip_open_by_name(chip.c_str());

    if (!this->chip) {
      LOG("LibGpiodController::init: gpiod_chip_open_by_name(" << chip << ") failed");
      return false;
    }

    this->startLineHandle = gpiod_chip_get_line(this->chip, startLine);
    this->dropLineHandle = gpiod_chip_get_line(this->chip, dropLine);

    if (!this->startLineHandle || !this->dropLineHandle) {
      LOG("LibGpiodController::init: gpiod_chip_get_line failed (startLine=" << startLine << " dropLine=" << dropLine << ")");
      this->release();
      return false;
    }

    if (gpiod_line_request_output(this->startLineHandle, "simulation11-start", 0) != 0) {
      LOG("LibGpiodController::init: gpiod_line_request_output(START) failed");
      this->release();
      return false;
    }
    if (gpiod_line_request_output(this->dropLineHandle, "simulation11-drop", 0) != 0) {
      LOG("LibGpiodController::init: gpiod_line_request_output(DROP) failed");
      this->release();
      return false;
    }

    LOG("LibGpiodController::init chip=" << chip << " startLine=" << startLine << " dropLine=" << dropLine << " OK (libgpiod v1 API)");
    return true;
  }

  void setStart(bool high) override
  {
    if (this->startLineHandle) {
      gpiod_line_set_value(this->startLineHandle, high ? 1 : 0);
    }
  }

  void setDrop(bool high) override
  {
    if (this->dropLineHandle) {
      gpiod_line_set_value(this->dropLineHandle, high ? 1 : 0);
    }
  }

private:
  void release()
  {
    if (this->startLineHandle) {
      gpiod_line_set_value(this->startLineHandle, 0);
      gpiod_line_release(this->startLineHandle);
      this->startLineHandle = nullptr;
    }
    if (this->dropLineHandle) {
      gpiod_line_set_value(this->dropLineHandle, 0);
      gpiod_line_release(this->dropLineHandle);
      this->dropLineHandle = nullptr;
    }
    if (this->chip) {
      gpiod_chip_close(this->chip);
      this->chip = nullptr;
    }
  }

  gpiod_chip *chip{nullptr};
  gpiod_line *startLineHandle{nullptr};
  gpiod_line *dropLineHandle{nullptr};
};

}  // namespace gpio

#endif  // defined(USE_GPIOD)
