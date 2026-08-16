#pragma once

#include <cstdint>
#include <string>

class I2CDevice {
private:
  int fd{-1};
  uint8_t address;
  std::string device;

public:
  I2CDevice(std::string i2c_device, uint8_t i2c_address);
  virtual ~I2CDevice();
  virtual uint8_t identify();
  virtual void init();
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegister(uint8_t reg, uint8_t *buf, int length);
};