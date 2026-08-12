
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdint>
#include <iostream>

constexpr auto I2C_SLAVE = 0x0703;

int main(int argc, char** argv)
{
  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " <i2c device>" << " <i2c address>" << std::endl;
    return 1;
  }

  auto device = argv[1];
  auto address = std::stoul(argv[2], nullptr, 16);

  auto fd = open(device, O_RDWR);
  ioctl(fd, I2C_SLAVE, address);

  uint8_t reg = 0x75;  // WHO_AM_I register
  write(fd, &reg, 1);

  uint8_t value = 0;
  read(fd, &value, 1);

  std::cout << "Value read from register 0x" << std::hex << static_cast<int>(reg) << ": 0x" << std::hex << static_cast<int>(value)
            << std::endl;
  close(fd);
  return 0;
}