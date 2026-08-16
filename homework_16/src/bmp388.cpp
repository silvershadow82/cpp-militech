#include "bmp388.h"

#include <cstdint>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace {

enum class Oversampling : uint8_t {
  x1 = 0,
  x2 = 1,
  x4 = 2,
  x8 = 3,
  x16 = 4,
  x32 = 5,
};

// Регістри BMP388 віддають дані little-endian, тому параметри названо за порядком
// значущості байтів, а не за порядком читання.
uint16_t toLittleEndianUnsigned16(uint8_t lsb, uint8_t msb)
{
  return (static_cast<uint16_t>(msb) << 8) | lsb;
}

int16_t toLittleEndianSigned16(uint8_t lsb, uint8_t msb)
{
  return static_cast<int16_t>(toLittleEndianUnsigned16(lsb, msb));
}

uint32_t toLittleEndianUnsigned24(uint8_t msb, uint8_t lsb, uint8_t xlsb)
{
  return (static_cast<uint32_t>(msb) << 16) | (static_cast<uint32_t>(lsb) << 8) | xlsb;
}

}  // namespace

uint8_t BMP388::identify()
{
  uint8_t value = 0;
  if (this->readRegister(BMP388_REGISTER_CHIP_ID, &value, 1)) {
    return value;
  }
  return 0;
}

void BMP388::readCalibrationData()
{
  uint8_t raw_data[BMP388_CALIBRATION_DATA_SIZE];
  if (this->readRegister(BMP388_REGISTER_CALIBRATION_DATA, raw_data, sizeof(raw_data))) {
    this->calibration = this->quantizeCalibrationData(raw_data);
  }
}

void BMP388::init()
{
  I2CDevice::init();

  uint8_t chip_id = 0;
  if (this->readRegister(BMP388_REGISTER_CHIP_ID, &chip_id, 1)) {
    if (chip_id != BMP388_CHIP_ID) {
      std::cerr << "BMP388 invalid chipID: " << chip_id << " vs expected " << BMP388_CHIP_ID << std::endl;
      return;
    }
  }

  this->reset();
  this->readCalibrationData();

  uint8_t osr_value = static_cast<uint8_t>((static_cast<uint8_t>(Oversampling::x1) << 3) | static_cast<uint8_t>(Oversampling::x8));

  this->writeRegister(BMP388_REGISTER_OSR, osr_value);
  this->writeRegister(BMP388_REGISTER_CONFIG, 0x00);
}

void BMP388::reset()
{
  this->writeRegister(BMP388_REGISTER_CMD, BMP388_CMD_SOFT_RESET);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

BMP388::Calibration BMP388::quantizeCalibrationData(const uint8_t raw[BMP388_CALIBRATION_DATA_SIZE])
{
  // temperature calibration data
  uint16_t t1_raw = toLittleEndianUnsigned16(raw[0], raw[1]);
  uint16_t t2_raw = toLittleEndianUnsigned16(raw[2], raw[3]);
  int8_t t3_raw = static_cast<int8_t>(raw[4]);

  // pressure calibration data
  int16_t p1_raw = toLittleEndianSigned16(raw[5], raw[6]);
  int16_t p2_raw = toLittleEndianSigned16(raw[7], raw[8]);
  int8_t p3_raw = static_cast<int8_t>(raw[9]);
  int8_t p4_raw = static_cast<int8_t>(raw[10]);

  uint16_t p5_raw = toLittleEndianUnsigned16(raw[11], raw[12]);
  uint16_t p6_raw = toLittleEndianUnsigned16(raw[13], raw[14]);
  int8_t p7_raw = static_cast<int8_t>(raw[15]);
  int8_t p8_raw = static_cast<int8_t>(raw[16]);

  int16_t p9_raw = toLittleEndianSigned16(raw[17], raw[18]);
  int8_t p10_raw = static_cast<int8_t>(raw[19]);
  int8_t p11_raw = static_cast<int8_t>(raw[20]);

  // From BMP388 datasheet
  Calibration c{};
  c.par_t1 = static_cast<double>(t1_raw) / std::pow(2.0, -8.0);
  c.par_t2 = static_cast<double>(t2_raw) / std::pow(2.0, 30.0);
  c.par_t3 = static_cast<double>(t3_raw) / std::pow(2.0, 48.0);

  c.par_p1 = (static_cast<double>(p1_raw) - std::pow(2.0, 14.0)) / std::pow(2.0, 20.0);
  c.par_p2 = (static_cast<double>(p2_raw) - std::pow(2.0, 14.0)) / std::pow(2.0, 29.0);
  c.par_p3 = static_cast<double>(p3_raw) / std::pow(2.0, 32.0);
  c.par_p4 = static_cast<double>(p4_raw) / std::pow(2.0, 37.0);
  c.par_p5 = static_cast<double>(p5_raw) / std::pow(2.0, -3.0);
  c.par_p6 = static_cast<double>(p6_raw) / std::pow(2.0, 6.0);
  c.par_p7 = static_cast<double>(p7_raw) / std::pow(2.0, 8.0);
  c.par_p8 = static_cast<double>(p8_raw) / std::pow(2.0, 15.0);
  c.par_p9 = static_cast<double>(p9_raw) / std::pow(2.0, 48.0);
  c.par_p10 = static_cast<double>(p10_raw) / std::pow(2.0, 48.0);
  c.par_p11 = static_cast<double>(p11_raw) / std::pow(2.0, 65.0);

  return c;
}

double BMP388::compensateTemperature(uint32_t uncomp_temp, const BMP388::Calibration &c)
{
  double partial1 = static_cast<double>(uncomp_temp) - c.par_t1;
  double partial2 = partial1 * c.par_t2;
  double t_lin = partial2 + (partial1 * partial1) * c.par_t3;
  return t_lin;
}

double BMP388::compensatePressure(uint32_t uncomp_pressure, const Calibration &c, double comp_temp)
{
  double partial1 = c.par_p6 * comp_temp;
  double partial2 = c.par_p7 * comp_temp * comp_temp;
  double partial3 = c.par_p8 * std::pow(comp_temp, 3.0);
  double out1 = c.par_p5 + partial1 + partial2 + partial3;

  partial1 = c.par_p2 * comp_temp;
  partial2 = c.par_p3 * comp_temp * comp_temp;
  partial3 = c.par_p4 * std::pow(comp_temp, 3.0);
  double up = static_cast<double>(uncomp_pressure);

  double out2 = up * (c.par_p1 + partial1 + partial2 + partial3);

  partial1 = up * up;
  partial2 = c.par_p9 + c.par_p10 * comp_temp;
  partial3 = partial1 * partial2;
  double out3 = partial3 + std::pow(up, 3.0) * c.par_p11;

  return out1 + out2 + out3;
}

Reading BMP388::readOnce()
{
  // Enable pressure and temp measurement in force mode
  this->writeRegister(BMP388_REGISTER_POWER,
                      BMP388_PWR_CTRL_ENABLE_PRESSURE | BMP388_PWR_CTRL_ENABLE_TEMPERATURE | BMP388_PWR_CTRL_MODE_FORCED);

  constexpr auto poll_interval = std::chrono::milliseconds(2);
  constexpr int max_polls = 250;

  bool ready = false;
  for (int i = 0; i < max_polls; ++i) {
    uint8_t status = 0;
    this->readRegister(BMP388_REGISTER_STATUS, &status, 1);
    if ((status & BMP388_STATUS_TEMP_READY) && (status & BMP388_STATUS_PRESSURE_READY)) {
      ready = true;
      break;
    }
    std::this_thread::sleep_for(poll_interval);
  }
  if (!ready) {
    uint8_t error = 0;
    this->readRegister(BMP388_REGISTER_ERR, &error, 1);
    throw std::runtime_error("BMP388 data not ready: " + std::to_string(error));
  }

  uint8_t raw[6];  // 3 temp + 3 pressure
  this->readRegister(BMP388_REGISTER_DATA_0, raw, sizeof(raw));

  // Обидва сирі значення — 24-бітні беззнакові (data_0..data_2 — тиск, data_3..data_5 —
  // температура), тому знакове перетворення тут не застосовується.
  uint32_t uncomp_pressure = toLittleEndianUnsigned24(raw[2], raw[1], raw[0]);
  uint32_t uncomp_temp = toLittleEndianUnsigned24(raw[5], raw[4], raw[3]);

  double temperature_c = this->compensateTemperature(uncomp_temp, this->calibration);
  double pressure_pa = this->compensatePressure(uncomp_pressure, this->calibration, temperature_c);

  return Reading{temperature_c, pressure_pa};
}