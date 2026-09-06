#pragma once

#include "i2c_device.h"
#include <cstddef>
#include <cstdint>

// BMP388 register data
#define BMP388_ADDRESS 0x77
#define BMP388_REGISTER_CHIP_ID 0x00
#define BMP388_REGISTER_ERR 0x02
#define BMP388_REGISTER_STATUS 0x03
#define BMP388_REGISTER_DATA_0 0x04
#define BMP388_REGISTER_CALIBRATION_DATA 0x31
#define BMP388_REGISTER_OSR 0x1C
#define BMP388_REGISTER_CONFIG 0x1F
#define BMP388_REGISTER_POWER 0x1B
#define BMP388_REGISTER_CMD 0x7E

#define BMP388_CHIP_ID 0x50
#define BMP388_CMD_SOFT_RESET 0xB6
#define BMP388_STATUS_PRESSURE_READY 1 << 5
#define BMP388_STATUS_TEMP_READY 1 << 6

#define BMP388_PWR_CTRL_ENABLE_PRESSURE 1 << 0
#define BMP388_PWR_CTRL_ENABLE_TEMPERATURE 1 << 1
#define BMP388_PWR_CTRL_MODE_FORCED 0b01 << 4

#define BMP388_CALIBRATION_DATA_SIZE 21

// Результат одного вимірювання. Тип з зовнішнім зв'язуванням: він є частиною публічного
// API BMP388::readOnce(), тому не може жити в анонімному просторі імен.
struct Reading {
  double temperature_c;
  double pressure_pa;
};

class BMP388 : public I2CDevice {
private:
  struct Calibration {
    double par_t1 = 0, par_t2 = 0, par_t3 = 0;
    double par_p1 = 0, par_p2 = 0, par_p3 = 0, par_p4 = 0, par_p5 = 0, par_p6 = 0, par_p7 = 0, par_p8 = 0, par_p9 = 0, par_p10 = 0,
           par_p11 = 0;
  };
  void reset();
  void readCalibrationData();
  static Calibration quantizeCalibrationData(const uint8_t raw_data[BMP388_CALIBRATION_DATA_SIZE]);
  static double compensateTemperature(uint32_t uncomp_temp, const Calibration &calibration);
  static double compensatePressure(uint32_t uncomp_pressure, const Calibration &calibration, double comp_temp);

  Calibration calibration{};

public:
  BMP388(std::string device, uint8_t address)
    : I2CDevice(device, address) {};
  Reading readOnce();
  uint8_t identify() override;
  void init() override;
};