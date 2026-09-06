#include "bmp388.h"
#include <cstdint>
#include <iomanip>
#include <iostream>

int main(int argc, char **argv)
{
  BMP388 device{"/dev/i2c-1", BMP388_ADDRESS};
  device.init();

  std::cout << "Identify from register 0x" << std::hex << BMP388_REGISTER_CHIP_ID << ": 0x" << std::hex
            << static_cast<unsigned int>(device.identify()) << std::endl;

  std::cout << "Starting readings..." << std::endl;

  Reading reading = device.readOnce();

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Temperature: " << reading.temperature_c << " C" << std::endl;
  std::cout << "Pressure: " << reading.pressure_pa << " Pa" << std::endl;

  return 0;
}