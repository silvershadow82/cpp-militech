#include "bmp388.h"
#include <cstdint>
#include <iostream>

int main(int argc, char **argv)
{
  BMP388 device{"/dev/i2c-1", BMP388_ADDRESS};
  device.init();

  std::cout << "Identify from register 0x" << std::hex << BMP388_REGISTER_CHIP_ID << ": 0x" << std::hex << device.identify() << std::endl;

  return 0;
}