#include "i2c_device.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <cstdint>
#include <iostream>

I2CDevice::I2CDevice(std::string i2c_device, uint8_t i2c_address)
  : address(i2c_address)
  , device(i2c_device)
{
}

// Загальна I2C-шина не має стандартного регістра ідентифікації — конкретні пристрої
// перевизначають цей метод.
uint8_t I2CDevice::identify()
{
  return 0;
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

bool I2CDevice::writeRegister(uint8_t reg, uint8_t value)
{
  if (this->fd < 0) {
    return false;
  }

  // I2C-запис регістра — це одна транзакція з двох байтів: адреса регістра, потім значення.
  const uint8_t frame[2] = {reg, value};
  return write(this->fd, frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}