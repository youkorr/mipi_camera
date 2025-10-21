#pragma once

#include "camera_config.h"
#include <cstdint>
#include <cstring>
#include <algorithm>

#ifdef USE_ESP32_VARIANT_ESP32P4
extern "C" {
  #include "driver/isp.h"
}
#endif

namespace esphome {
namespace mipi_camera {

/**
 * Gestion de la correction couleur (CCM + WB)
 */
class ColorCorrection {
public:
  enum class CCMProfile {
    INDOOR,
    LED_COOL,
    OUTDOOR,
    CUSTOM
  };

  ColorCorrection() = default;

  /**
   * Initialise la CCM matérielle (si disponible)
   * Note: CCM matériel non disponible sur ESP32-P4 actuellement
   */
  bool init_hardware_ccm(isp_proc_handle_t isp_handle, CCMProfile profile = CCMProfile::INDOOR) {
#ifdef USE_ESP32_VARIANT_ESP32P4
    // CCM matériel non supporté dans l'API ISP actuelle
    // On utilise uniquement le software WB
    (void)isp_handle;
    (void)profile;
    this->hardware_ccm_enabled_ = false;
    return false;
#else
    return false;
#endif
  }

  /**
   * Change le profil CCM à la volée
   */
  bool set_ccm_profile(CCMProfile profile) {
    // CCM matériel non supporté
    (void)profile;
    return false;
  }

  /**
   * Applique White Balance software sur un buffer RGB565
   * (fallback si AWB hardware échoue)
   */
  void apply_software_wb(uint8_t* buffer, size_t width, size_t height) {
    if (!buffer) return;

    uint16_t* pixels = reinterpret_cast<uint16_t*>(buffer);
    size_t pixel_count = width * height;

    // Gains en fixed-point (Q8.8)
    uint16_t r_gain_fp = static_cast<uint16_t>(white_balance::RED_GAIN * 256.0f);
    uint16_t g_gain_fp = static_cast<uint16_t>(white_balance::GREEN_GAIN * 256.0f);
    uint16_t b_gain_fp = static_cast<uint16_t>(white_balance::BLUE_GAIN * 256.0f);

    for (size_t i = 0; i < pixel_count; i++) {
      pixels[i] = apply_wb_to_pixel(pixels[i], r_gain_fp, g_gain_fp, b_gain_fp);
    }
  }

  /**
   * Applique WB seulement sur une région centrale (plus rapide)
   */
  void apply_software_wb_center(uint8_t* buffer, size_t width, size_t height, float region = 0.6f) {
    if (!buffer) return;

    size_t start_x = static_cast<size_t>(width * (1.0f - region) / 2.0f);
    size_t start_y = static_cast<size_t>(height * (1.0f - region) / 2.0f);
    size_t end_x = width - start_x;
    size_t end_y = height - start_y;

    uint16_t* pixels = reinterpret_cast<uint16_t*>(buffer);

    uint16_t r_gain_fp = static_cast<uint16_t>(white_balance::RED_GAIN * 256.0f);
    uint16_t g_gain_fp = static_cast<uint16_t>(white_balance::GREEN_GAIN * 256.0f);
    uint16_t b_gain_fp = static_cast<uint16_t>(white_balance::BLUE_GAIN * 256.0f);

    for (size_t y = start_y; y < end_y; y++) {
      for (size_t x = start_x; x < end_x; x++) {
        size_t idx = y * width + x;
        pixels[idx] = apply_wb_to_pixel(pixels[idx], r_gain_fp, g_gain_fp, b_gain_fp);
      }
    }
  }

  bool is_hardware_ccm_enabled() const { return this->hardware_ccm_enabled_; }

private:
  const float* get_ccm_matrix(CCMProfile profile) const {
    switch (profile) {
      case CCMProfile::INDOOR:
        return color_correction::CCM_INDOOR;
      case CCMProfile::LED_COOL:
        return color_correction::CCM_LED_COOL;
      case CCMProfile::OUTDOOR:
        return color_correction::CCM_OUTDOOR;
      default:
        return color_correction::CCM_INDOOR;
    }
  }

  inline uint16_t apply_wb_to_pixel(uint16_t pixel, uint16_t r_gain, uint16_t g_gain, uint16_t b_gain) {
    // Extraire RGB565
    uint8_t r = (pixel >> 11) & 0x1F;
    uint8_t g = (pixel >> 5) & 0x3F;
    uint8_t b = pixel & 0x1F;

    // Appliquer gains
    uint16_t r_new = (r * r_gain) >> 8;
    uint16_t g_new = (g * g_gain) >> 8;
    uint16_t b_new = (b * b_gain) >> 8;

    // Clamper
    r_new = std::min<uint16_t>(r_new, 0x1F);
    g_new = std::min<uint16_t>(g_new, 0x3F);
    b_new = std::min<uint16_t>(b_new, 0x1F);

    return (r_new << 11) | (g_new << 5) | b_new;
  }

  bool hardware_ccm_enabled_{false};
};

}  // namespace mipi_camera
}  // namespace esphome
