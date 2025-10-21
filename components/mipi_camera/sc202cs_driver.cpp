#include "sc202cs_driver.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mipi_camera {
namespace sensors {

static const char *const TAG = "sc202cs";

// ==================================================
// REGISTRES SUPPLÉMENTAIRES POUR FLIP/MIRROR
// ==================================================

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
  
  // ✅ NOUVEAU: Registre de flip/mirror
  constexpr uint16_t FLIP_MIRROR = 0x3221;
  // Bits du registre 0x3221:
  // Bit 1: Mirror (0=normal, 1=miroir horizontal)
  // Bit 2: Flip (0=normal, 1=flip vertical)
}

// ... (le reste du code existant reste identique) ...

// ==================================================
// NOUVELLES FONCTIONS FLIP/MIRROR
// ==================================================

esp_err_t SC202CSDriver::set_flip(bool enable) {
  uint8_t current_value = 0;
  
  // Lire la valeur actuelle
  esp_err_t ret = read_register(Registers::FLIP_MIRROR, &current_value);
  if (ret != ESP_OK) return ret;
  
  // Modifier le bit de flip (bit 2)
  if (enable) {
    current_value |= 0x04;  // Set bit 2
  } else {
    current_value &= ~0x04; // Clear bit 2
  }
  
  ret = write_register(Registers::FLIP_MIRROR, current_value);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Flip vertical: %s", enable ? "ON" : "OFF");
  }
  
  return ret;
}

esp_err_t SC202CSDriver::set_mirror(bool enable) {
  uint8_t current_value = 0;
  
  // Lire la valeur actuelle
  esp_err_t ret = read_register(Registers::FLIP_MIRROR, &current_value);
  if (ret != ESP_OK) return ret;
  
  // Modifier le bit de mirror (bit 1)
  if (enable) {
    current_value |= 0x02;  // Set bit 1
  } else {
    current_value &= ~0x02; // Clear bit 1
  }
  
  ret = write_register(Registers::FLIP_MIRROR, current_value);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Mirror horizontal: %s", enable ? "ON" : "OFF");
  }
  
  return ret;
}

bool SC202CSDriver::supports_flip_mirror() const {
  return true;  // ✅ SC202CS supporte flip/mirror
}

// ... (reste du code inchangé) ...

}  // namespace sensors
}  // namespace mipi_camera
}  // namespace esphome
