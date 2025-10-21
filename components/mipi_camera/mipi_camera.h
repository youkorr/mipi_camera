#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

#include "camera_config.h"
#include "color_correction.h"
#include "sensor_driver.h"

#include <string>
#include <atomic>

#ifdef USE_ESP32_VARIANT_ESP32P4
extern "C" {
  #include "esp_cam_ctlr.h"
  #include "esp_cam_ctlr_csi.h"
  #include "driver/isp.h"
  #include "esp_ldo_regulator.h"
  // ❌ RETIRÉ: #include "driver/ppa.h"
}
#endif

namespace esphome {
namespace mipi_camera {

/**
 * Composant principal de caméra MIPI
 * Compatible ESPHome, extensible pour plusieurs capteurs
 * Mirror/Flip gérés directement par le capteur
 */
class MIPICameraComponent : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Configuration depuis YAML
  void set_name(const std::string &name) { this->name_ = name; }
  void set_sensor_type(const std::string &type) { this->sensor_type_ = type; }
  void set_external_clock_pin(int8_t pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->external_clock_freq_ = freq; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_resolution(const std::string &res) { this->resolution_ = res; }
  void set_pixel_format(const std::string &format) { this->pixel_format_ = format; }
  void set_framerate(uint8_t fps) { this->framerate_ = fps; }
  void set_jpeg_quality(uint8_t quality) { this->jpeg_quality_ = quality; }
  void set_mirror_enabled(bool enable) { this->mirror_enabled_ = enable; }
  void set_flip_enabled(bool enable) { this->flip_enabled_ = enable; }

  // API Camera - Streaming
  bool start_streaming();
  bool stop_streaming();
  bool is_streaming() const { return this->streaming_; }
  
  // API Camera - Capture
  bool capture_frame();
  bool has_new_frame() const { return this->display_buffer_ready_.load(); }
  
  // API Camera - Données image
  uint8_t* get_image_data() { 
    return this->display_buffer_; 
  }
  
  uint16_t get_image_width() const;
  uint16_t get_image_height() const;
  
  size_t get_image_size() const { 
    return this->frame_buffer_size_; 
  }
  
  uint32_t get_frame_number() const { 
    return this->frame_number_.load(); 
  }

  // API Camera - Contrôle exposition/gain
  void set_auto_exposure(bool enabled);
  void set_manual_exposure(uint16_t exposure);
  void set_manual_gain(uint8_t gain_index);
  void set_brightness_level(uint8_t level);  // 0-10

  // API Camera - Profils couleur
  void set_color_profile(ColorCorrection::CCMProfile profile);
  
  // API Camera - Flip et Mirror (runtime via capteur)
  void set_flip(bool enable);
  void set_mirror(bool enable);
  
  // État
  bool is_initialized() const { return this->initialized_; }

 protected:
  // Configuration de base
  std::string name_{""};
  std::string sensor_type_{""};
  int8_t external_clock_pin_{-1};
  uint32_t external_clock_freq_{24000000};
  GPIOPin *reset_pin_{nullptr};
  
  // Configuration additionnelle
  std::string resolution_{"720P"};
  std::string pixel_format_{"RGB565"};
  uint8_t framerate_{30};
  uint8_t jpeg_quality_{10};
  bool mirror_enabled_{true};   // Mirror par défaut via capteur
  bool flip_enabled_{false};

  // État
  bool initialized_{false};
  bool streaming_{false};

  // Triple buffering
  static constexpr size_t NUM_BUFFERS = performance::NUM_BUFFERS;
  uint8_t *frame_buffers_[NUM_BUFFERS]{nullptr};
  std::atomic<uint8_t> capture_buffer_index_{0};
  std::atomic<uint8_t> ready_buffer_index_{1};
  uint8_t *display_buffer_{nullptr};
  
  std::atomic<bool> display_buffer_ready_{false};
  std::atomic<uint32_t> frame_number_{0};
  size_t frame_buffer_size_{0};

  // Stats
  uint32_t total_frames_received_{0};
  uint32_t last_frame_log_time_{0};

  // Driver et corrections
  ISensorDriver *sensor_driver_{nullptr};
  ColorCorrection color_correction_;

  // Auto-exposition
  bool auto_exposure_enabled_{auto_exposure::ENABLED_BY_DEFAULT};
  uint16_t current_exposure_{0};
  uint8_t current_gain_index_{0};
  uint32_t last_ae_update_{0};
  
  struct AECommand {
    uint16_t exposure;
    uint8_t gain;
  };
  QueueHandle_t ae_command_queue_{nullptr};
  TaskHandle_t ae_task_handle_{nullptr};

#ifdef USE_ESP32_VARIANT_ESP32P4
  esp_cam_ctlr_handle_t csi_handle_{nullptr};
  isp_proc_handle_t isp_handle_{nullptr};
  esp_ldo_channel_handle_t ldo_handle_{nullptr};
  isp_awb_ctlr_t awb_ctlr_{nullptr};
  
  // ❌ RETIRÉ: Variables PPA

  // Initialisation
  bool create_sensor_driver_();
  bool init_sensor_();
  bool init_external_clock_();
  bool init_ldo_();
  bool init_csi_();
  bool init_isp_();
  bool allocate_buffers_();
  void configure_color_correction_();
  void configure_awb_();

  // Auto-exposition
  void update_auto_exposure_();
  uint32_t calculate_brightness_();
  static void ae_task_(void* param);

  // ISR callbacks
  static bool IRAM_ATTR on_csi_new_frame_(
    esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data
  );
  
  static bool IRAM_ATTR on_csi_frame_done_(
    esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data
  );
#endif
};

}  // namespace mipi_camera
}  // namespace esphome
