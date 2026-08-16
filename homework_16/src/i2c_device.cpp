#include "i2c_device.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <cstdint>
#include <iostream>

I2CDevice::I2CDevice(std::string i2c_device, uint8_t i2c_address)
  : device(i2c_device)
  , address(i2c_address)
{
}

void I2CDevice::init()
{
  this->fd = open(this->device.c_str(), O_RDWR);
  if (this->fd < 0) {
    std::cerr << "Unable to open device " << this->device << std::endl;
    return;
  }
  ioctl(this->fd, I2C_SLAVE, this->address);
}

I2CDevice::~I2CDevice()
{
  if (this->fd >= 0) {
    close(this->fd);
  }
}

bool I2CDevice::readRegister(uint8_t reg, uint8_t *value, int length)
{
  if (this->fd < 0) {
    return false;
  }

  if (write(this->fd, &reg, 1) != 1) {
    return false;
  }

  return read(this->fd, value, length) == length;
}

bool I2CDevice::writeRegister(u_int8_t reg, uint8_t value)
{
  if (this->fd < 0) {
    return false;
  }

  return write(this->fd, &reg, value) == sizeof(value);
}