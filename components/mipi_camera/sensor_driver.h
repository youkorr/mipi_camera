#pragma once

#include <cstdint>
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace mipi_camera {

/**
 * Informations du capteur
 */
struct SensorInfo {
  const char* name;
  const char* manufacturer;
  uint16_t pid;
  uint8_t i2c_address;
  uint8_t lane_count;
  uint8_t bayer_pattern;
  uint16_t lane_bitrate_mbps;
  uint16_t width;
  uint16_t height;
  uint8_t fps;
  
  // Limites exposition/gain par défaut
  uint16_t default_exposure;
  uint16_t min_exposure;
  uint16_t max_exposure;
  uint8_t default_gain_index;
  uint8_t min_gain_index;
  uint8_t max_gain_index;
};

/**
 * Interface abstraite pour tous les drivers de capteurs
 */
class ISensorDriver {
public:
  virtual ~ISensorDriver() = default;

  // Informations du capteur
  virtual const SensorInfo& get_info() const = 0;
  
  // Initialisation
  virtual esp_err_t init() = 0;
  virtual esp_err_t read_id(uint16_t* pid) = 0;
  
  // Contrôle stream
  virtual esp_err_t start_stream() = 0;
  virtual esp_err_t stop_stream() = 0;
  
  // Contrôle exposition/gain
  virtual esp_err_t set_exposure(uint16_t exposure) = 0;
  virtual esp_err_t set_gain(uint8_t gain_index) = 0;
  
  // Accès registres (pour debug)
  virtual esp_err_t write_register(uint16_t reg, uint8_t value) = 0;
  virtual esp_err_t read_register(uint16_t reg, uint8_t* value) = 0;
  
  // Fonctionnalités optionnelles
  virtual bool supports_test_pattern() const { return false; }
  virtual esp_err_t set_test_pattern(uint8_t mode) { return ESP_ERR_NOT_SUPPORTED; }
  
  virtual bool supports_flip_mirror() const { return false; }
  virtual esp_err_t set_flip(bool enable) { return ESP_ERR_NOT_SUPPORTED; }
  virtual esp_err_t set_mirror(bool enable) { return ESP_ERR_NOT_SUPPORTED; }
};

/**
 * Classe de base pour simplifier l'implémentation des drivers
 */
class SensorDriverBase : public ISensorDriver {
public:
  SensorDriverBase(i2c::I2CDevice* i2c, const SensorInfo& info) 
    : i2c_(i2c), info_(info) {}

  const SensorInfo& get_info() const override { return info_; }

  // Implémentation I2C commune
  esp_err_t write_register(uint16_t reg, uint8_t value) override {
    uint8_t data[3] = {
      static_cast<uint8_t>((reg >> 8) & 0xFF),
      static_cast<uint8_t>(reg & 0xFF),
      value
    };
    
    auto err = i2c_->write_read(data, 3, nullptr, 0);
    return (err == i2c::ERROR_OK) ? ESP_OK : ESP_FAIL;
  }

  esp_err_t read_register(uint16_t reg, uint8_t* value) override {
    uint8_t addr[2] = {
      static_cast<uint8_t>((reg >> 8) & 0xFF),
      static_cast<uint8_t>(reg & 0xFF)
    };
    
    auto err = i2c_->write_read(addr, 2, value, 1);
    return (err == i2c::ERROR_OK) ? ESP_OK : ESP_FAIL;
  }

protected:
  i2c::I2CDevice* i2c_;
  const SensorInfo& info_;
};

/**
 * Factory pour créer les drivers
 */
ISensorDriver* create_sensor_driver(const std::string& sensor_type, i2c::I2CDevice* i2c);

}  // namespace mipi_camera
}  // namespace esphome
