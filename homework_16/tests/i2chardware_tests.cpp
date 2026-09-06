#include "bmp388.h"
#include "i2c_device.h"

#include <cstdint>
#include <stdexcept>

#include <gtest/gtest.h>

namespace {
// Шлях свідомо не існує: тести працюють без підключеного BMP388 і без /dev/i2c-*,
// тому перевіряють лише поведінку "пристрій недоступний" (fd < 0), а не саму шину.
constexpr auto kMissingDevicePath = "/dev/i2chardware-tests-missing";
}  // namespace

TEST(BMP388Constants, ChipIdMatchesDatasheet)
{
  EXPECT_EQ(BMP388_CHIP_ID, 0x50);
}

TEST(BMP388Constants, DefaultAddressMatchesDatasheet)
{
  EXPECT_EQ(BMP388_ADDRESS, 0x77);
}

TEST(I2CDevice, DefaultIdentifyReturnsZero)
{
  I2CDevice device{kMissingDevicePath, 0x77};
  EXPECT_EQ(device.identify(), 0);
}

TEST(I2CDevice, ReadRegisterFailsWithoutOpenDevice)
{
  I2CDevice device{kMissingDevicePath, 0x77};
  uint8_t value = 0xAB;
  EXPECT_FALSE(device.readRegister(0x00, &value, 1));
}

TEST(I2CDevice, WriteRegisterFailsWithoutOpenDevice)
{
  I2CDevice device{kMissingDevicePath, 0x77};
  EXPECT_FALSE(device.writeRegister(0x00, 0x00));
}

TEST(I2CDevice, InitOnMissingDevicePathDoesNotCrash)
{
  I2CDevice device{kMissingDevicePath, 0x77};
  device.init();

  uint8_t value = 0;
  EXPECT_FALSE(device.readRegister(0x00, &value, 1));
}

TEST(BMP388, IdentifyReturnsZeroWithoutOpenDevice)
{
  BMP388 device{kMissingDevicePath, BMP388_ADDRESS};
  EXPECT_EQ(device.identify(), 0);
}

TEST(BMP388, InitDoesNotCrashWhenDeviceMissing)
{
  BMP388 device{kMissingDevicePath, BMP388_ADDRESS};
  EXPECT_NO_THROW(device.init());
  EXPECT_EQ(device.identify(), 0);
}

TEST(BMP388, ReadOnceThrowsWhenDeviceMissing)
{
  BMP388 device{kMissingDevicePath, BMP388_ADDRESS};
  device.init();
  EXPECT_THROW(device.readOnce(), std::runtime_error);
}
