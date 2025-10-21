#pragma once

#include "sensor_driver.h"

namespace esphome {
namespace mipi_camera {
namespace sensors {

/**
 * Registres SC202CS
 */
namespace Registers {
  constexpr uint16_t SENSOR_ID_H = 0x3107;
  constexpr uint16_t SENSOR_ID_L = 0x3108;
  constexpr uint16_t STREAM_MODE = 0x0100;
  constexpr uint16_t EXPOSURE_H = 0x3e00;
  constexpr uint16_t EXPOSURE_M = 0x3e01;
  constexpr uint16_t EXPOSURE_L = 0x3e02;
  constexpr uint16_t GAIN_ANALOG = 0x3e09;
  constexpr uint16_t GAIN_FINE = 0x3e07;
  constexpr uint16_t GAIN_COARSE = 0x3e06;
  constexpr uint16_t FLIP_MIRROR = 0x3221;  // Flip/Mirror control
}

/**
 * Driver SC202CS pour ESP32-P4
 */
class SC202CSDriver : public SensorDriverBase {
public:
  struct InitRegister {
    uint16_t addr;
    uint8_t value;
    uint16_t delay_ms;
  };

  struct GainRegisters {
    uint8_t fine;
    uint8_t coarse;
    uint8_t analog;
  };

  explicit SC202CSDriver(i2c::I2CDevice* i2c);

  // Interface ISensorDriver
  esp_err_t init() override;
  esp_err_t read_id(uint16_t* pid) override;
  esp_err_t start_stream() override;
  esp_err_t stop_stream() override;
  esp_err_t set_exposure(uint16_t exposure) override;
  esp_err_t set_gain(uint8_t gain_index) override;

  // Support flip/mirror
  bool supports_flip_mirror() const override { return true; }
  esp_err_t set_flip(bool enable) override;
  esp_err_t set_mirror(bool enable) override;

  bool supports_test_pattern() const override { return false; }

private:
  static const SensorInfo INFO;
  static const InitRegister INIT_SEQUENCE[];
  static const GainRegisters GAIN_TABLE[];
};

}  // namespace sensors
}  // namespace mipi_camera
}  // namespace esphome
