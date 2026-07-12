#pragma once

#include <string>

namespace gpio {

class IGpioController {
public:
  virtual ~IGpioController() = default;
  virtual bool init(const std::string& chip, unsigned startLine, unsigned dropLine) = 0;
  virtual void setStart(bool high) = 0;
  virtual void setDrop(bool high) = 0;
};

}  // namespace gpio
