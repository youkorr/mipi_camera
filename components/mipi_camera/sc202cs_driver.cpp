#include "mipi_camera.h"
#include "sc202cs_driver.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#ifdef USE_ESP32_VARIANT_ESP32P4
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#endif

namespace esphome {
namespace mipi_camera {

static const char *const TAG = "mipi_camera";

// ==================================================
// SETUP & INITIALIZATION
// ==================================================

void MIPICameraComponent::setup() {
  if (debug::VERBOSE_INIT) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "MIPI Camera System v2.0 + PPA");
    if (!this->name_.empty()) {
      ESP_LOGI(TAG, "  Name: %s", this->name_.c_str());
    }
    ESP_LOGI(TAG, "  Sensor: %s", this->sensor_type_.c_str());
    ESP_LOGI(TAG, "  Resolution: %s", this->resolution_.c_str());
    ESP_LOGI(TAG, "  Pixel Format: %s", this->pixel_format_.c_str());
    ESP_LOGI(TAG, "  Framerate: %d fps", this->framerate_);
    ESP_LOGI(TAG, "  Mirror: %s (PPA)", this->mirror_enabled_ ? "ON" : "OFF");
    ESP_LOGI(TAG, "  Flip: %s (PPA)", this->flip_enabled_ ? "ON" : "OFF");
    ESP_LOGI(TAG, "  Triple buffering: %d buffers", NUM_BUFFERS);
    ESP_LOGI(TAG, "  Auto Exposure: %s", auto_exposure::ENABLED_BY_DEFAULT ? "ON" : "OFF");
    ESP_LOGI(TAG, "========================================");
  }

  // Reset pin
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(20);
  }

  // Créer le driver de capteur
  if (!this->create_sensor_driver_()) {
    ESP_LOGE(TAG, "❌ Échec création driver");
    this->mark_failed();
    return;
  }

  // Initialiser le capteur
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "❌ Échec init sensor");
    this->mark_failed();
    return;
  }

  // Horloge externe (optionnel)
  if (this->external_clock_pin_ >= 0) {
    if (!this->init_external_clock_()) {
      ESP_LOGE(TAG, "❌ Échec init clock");
      this->mark_failed();
      return;
    }
  }

  // LDO MIPI
  if (!this->init_ldo_()) {
    ESP_LOGE(TAG, "❌ Échec init LDO");
    this->mark_failed();
    return;
  }

  // CSI
  if (!this->init_csi_()) {
    ESP_LOGE(TAG, "❌ Échec init CSI");
    this->mark_failed();
    return;
  }

  // ISP + Corrections couleur
  if (!this->init_isp_()) {
    ESP_LOGE(TAG, "❌ Échec init ISP");
    this->mark_failed();
    return;
  }

  // Buffers
  if (!this->allocate_buffers_()) {
    ESP_LOGE(TAG, "❌ Échec allocation buffers");
    this->mark_failed();
    return;
  }

  // ✅ PPA pour mirror/flip hardware
  if (!this->init_ppa_()) {
    ESP_LOGW(TAG, "⚠️  PPA non disponible, mirror désactivé");
    this->use_ppa_mirror_ = false;
  }

  // Tâche AE asynchrone
  this->ae_command_queue_ = xQueueCreate(4, sizeof(AECommand));
  if (!this->ae_command_queue_) {
    ESP_LOGE(TAG, "❌ Échec création queue AE");
    this->mark_failed();
    return;
  }

  xTaskCreatePinnedToCore(
    ae_task_,
    "ae_task",
    3072,
    this,
    1,  // Priorité basse
    &this->ae_task_handle_,
    0   // Core 0
  );

  this->initialized_ = true;
  
  if (debug::VERBOSE_INIT) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "✅ Caméra prête (%dx%d)", 
             this->get_image_width(), this->get_image_height());
    ESP_LOGI(TAG, "========================================");
  }
}

bool MIPICameraComponent::create_sensor_driver_() {
  // Factory pour créer les drivers
  if (this->sensor_type_ == "sc202cs") {
    this->sensor_driver_ = new sensors::SC202CSDriver(this);
    ESP_LOGI(TAG, "✓ Driver SC202CS créé");
    return true;
  }
  
  ESP_LOGE(TAG, "Capteur inconnu: %s", this->sensor_type_.c_str());
  return false;
}

bool MIPICameraComponent::init_sensor_() {
  if (!this->sensor_driver_) return false;

  const auto& info = this->sensor_driver_->get_info();
  
  ESP_LOGI(TAG, "Init sensor: %s (%s)", info.name, info.manufacturer);
  ESP_LOGI(TAG, "  Résolution: %dx%d @ %d fps", info.width, info.height, info.fps);
  ESP_LOGI(TAG, "  Lanes: %d, Bitrate: %d Mbps", info.lane_count, info.lane_bitrate_mbps);
  ESP_LOGI(TAG, "  Bayer: %d", info.bayer_pattern);

  // Lire ID
  uint16_t pid = 0;
  esp_err_t ret = this->sensor_driver_->read_id(&pid);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec lecture ID");
    return false;
  }

  if (pid != info.pid) {
    ESP_LOGE(TAG, "PID incorrect: 0x%04X (attendu 0x%04X)", pid, info.pid);
    return false;
  }

  ESP_LOGI(TAG, "✓ Sensor ID: 0x%04X", pid);

  // Initialiser
  ret = this->sensor_driver_->init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec init sensor");
    return false;
  }

  // Initialiser paramètres AE
  this->current_exposure_ = info.default_exposure;
  this->current_gain_index_ = info.default_gain_index;

  // ✅ Appliquer mirror/flip dans le capteur (plus simple que PPA)
  if (this->sensor_driver_->supports_flip_mirror()) {
    if (this->mirror_enabled_) {
      ESP_LOGI(TAG, "Activation mirror horizontal (capteur)...");
      ret = this->sensor_driver_->set_mirror(true);
      if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Mirror activé");
      } else {
        ESP_LOGW(TAG, "⚠️  Échec mirror: 0x%x", ret);
      }
      delay(50);
    }
    
    if (this->flip_enabled_) {
      ESP_LOGI(TAG, "Activation flip vertical (capteur)...");
      ret = this->sensor_driver_->set_flip(true);
      if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Flip activé");
      } else {
        ESP_LOGW(TAG, "⚠️  Échec flip: 0x%x", ret);
      }
      delay(50);
    }
  }

  delay(200);  // Stabilisation
  ESP_LOGI(TAG, "✓ Sensor initialisé et stable");

  return true;
}
bool MIPICameraComponent::init_external_clock_() {
#ifdef USE_ESP32_VARIANT_ESP32P4
  ESP_LOGI(TAG, "Init horloge externe GPIO%d @ %u Hz", 
           this->external_clock_pin_, this->external_clock_freq_);

  ledc_timer_config_t ledc_timer = {};
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer.duty_resolution = LEDC_TIMER_1_BIT;
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.freq_hz = this->external_clock_freq_;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;

  esp_err_t ret = ledc_timer_config(&ledc_timer);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec config LEDC timer");
    return false;
  }

  ledc_channel_config_t ledc_channel = {};
  ledc_channel.gpio_num = this->external_clock_pin_;
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.channel = LEDC_CHANNEL_0;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel.duty = 1;
  ledc_channel.hpoint = 0;

  ret = ledc_channel_config(&ledc_channel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec config LEDC channel");
    return false;
  }

  ESP_LOGI(TAG, "✓ Horloge externe OK");
#endif
  return true;
}

bool MIPICameraComponent::init_ldo_() {
#ifdef USE_ESP32_VARIANT_ESP32P4
  ESP_LOGI(TAG, "Init LDO MIPI");

  esp_ldo_channel_config_t ldo_config = {
    .chan_id = 3,
    .voltage_mv = 2500,
  };

  esp_err_t ret = esp_ldo_acquire_channel(&ldo_config, &this->ldo_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec LDO: %d", ret);
    return false;
  }

  ESP_LOGI(TAG, "✓ LDO OK (2.5V)");
#endif
  return true;
}

bool MIPICameraComponent::init_csi_() {
#ifdef USE_ESP32_VARIANT_ESP32P4
  ESP_LOGI(TAG, "Init MIPI-CSI");

  const auto& info = this->sensor_driver_->get_info();

  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.clk_src = MIPI_CSI_PHY_CLK_SRC_DEFAULT;
  csi_config.h_res = info.width;
  csi_config.v_res = info.height;
  csi_config.lane_bit_rate_mbps = info.lane_bitrate_mbps;
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.data_lane_num = info.lane_count;
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 10;

  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec CSI: %d", ret);
    return false;
  }

  // Callbacks
  esp_cam_ctlr_evt_cbs_t callbacks = {
    .on_get_new_trans = MIPICameraComponent::on_csi_new_frame_,
    .on_trans_finished = MIPICameraComponent::on_csi_frame_done_,
  };

  ret = esp_cam_ctlr_register_event_callbacks(this->csi_handle_, &callbacks, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec callbacks CSI");
    return false;
  }

  ret = esp_cam_ctlr_enable(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec enable CSI");
    return false;
  }

  ESP_LOGI(TAG, "✓ CSI OK");
#endif
  return true;
}

bool MIPICameraComponent::init_isp_() {
#ifdef USE_ESP32_VARIANT_ESP32P4
  ESP_LOGI(TAG, "Init ISP");

  const auto& info = this->sensor_driver_->get_info();

  esp_isp_processor_cfg_t isp_config = {};
  isp_config.clk_src = ISP_CLK_SRC_DEFAULT;
  isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
  isp_config.input_data_color_type = ISP_COLOR_RAW8;
  isp_config.output_data_color_type = ISP_COLOR_RGB565;
  isp_config.h_res = info.width;
  isp_config.v_res = info.height;
  isp_config.has_line_start_packet = false;
  isp_config.has_line_end_packet = false;
  isp_config.clk_hz = performance::ISP_CLOCK_HZ;
  isp_config.bayer_order = (color_raw_element_order_t)info.bayer_pattern;

  esp_err_t ret = esp_isp_new_processor(&isp_config, &this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec création ISP: 0x%x", ret);
    return false;
  }

  ret = esp_isp_enable(this->isp_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec enable ISP: 0x%x", ret);
    esp_isp_del_processor(this->isp_handle_);
    this->isp_handle_ = nullptr;
    return false;
  }

  // Configurer corrections couleur
  this->configure_color_correction_();
  this->configure_awb_();

  ESP_LOGI(TAG, "✓ ISP OK");
#endif
  return true;
}

void MIPICameraComponent::configure_color_correction_() {
#ifdef USE_ESP32_VARIANT_ESP32P4
  bool ccm_ok = this->color_correction_.init_hardware_ccm(
    this->isp_handle_, 
    ColorCorrection::CCMProfile::INDOOR
  );

  if (ccm_ok) {
    ESP_LOGI(TAG, "✓ CCM matérielle activée");
  } else {
    ESP_LOGW(TAG, "CCM matérielle non disponible → WB software actif");
  }
#endif
}

void MIPICameraComponent::configure_awb_() {
#ifdef USE_ESP32_VARIANT_ESP32P4
  // AWB matériel désactivé (limitations ESP32-P4)
  ESP_LOGW(TAG, "AWB matériel désactivé - utilisation exclusive du WB software");
  this->awb_ctlr_ = nullptr;
#endif
}

bool MIPICameraComponent::allocate_buffers_() {
#ifdef USE_ESP32_VARIANT_ESP32P4
  const auto& info = this->sensor_driver_->get_info();
  this->frame_buffer_size_ = info.width * info.height * 2;  // RGB565

  for (size_t i = 0; i < NUM_BUFFERS; i++) {
    this->frame_buffers_[i] = (uint8_t*)heap_caps_aligned_alloc(
      performance::BUFFER_ALIGNMENT,
      this->frame_buffer_size_,
      MALLOC_CAP_SPIRAM
    );

    if (!this->frame_buffers_[i]) {
      ESP_LOGE(TAG, "Échec allocation buffer %d", i);
      return false;
    }
  }

  this->display_buffer_ = this->frame_buffers_[2];

  ESP_LOGI(TAG, "✓ Buffers: %dx%u bytes", NUM_BUFFERS, this->frame_buffer_size_);
#endif
  return true;
}

bool MIPICameraComponent::init_ppa_() {
#ifdef USE_ESP32_VARIANT_ESP32P4
  ESP_LOGI(TAG, "Init PPA pour mirror/flip");

  ppa_client_config_t ppa_config = {
    .oper_type = PPA_OPERATION_SRM,  // Scale, Rotate, Mirror
    .max_pending_trans_num = 1,
  };

  esp_err_t ret = ppa_register_client(&ppa_config, &this->ppa_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec init PPA: 0x%x", ret);
    return false;
  }

  // Allouer buffer de sortie PPA (même taille que frame buffer)
  this->ppa_output_buffer_ = (uint8_t*)heap_caps_aligned_alloc(
    performance::BUFFER_ALIGNMENT,
    this->frame_buffer_size_,
    MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA
  );

  if (!this->ppa_output_buffer_) {
    ESP_LOGE(TAG, "Échec allocation buffer PPA");
    ppa_unregister_client(this->ppa_handle_);
    this->ppa_handle_ = nullptr;
    return false;
  }

  ESP_LOGI(TAG, "✓ PPA OK (SRM mode, mirror=%s, flip=%s)",
           this->mirror_enabled_ ? "ON" : "OFF",
           this->flip_enabled_ ? "ON" : "OFF");
#endif
  return true;
}

// ==================================================
// STREAMING CONTROL
// ==================================================

bool MIPICameraComponent::start_streaming() {
  if (!this->initialized_ || this->streaming_) {
    return false;
  }

  ESP_LOGI(TAG, "Démarrage streaming...");

  this->total_frames_received_ = 0;
  this->last_frame_log_time_ = millis();

  // Démarrer sensor
  if (this->sensor_driver_) {
    esp_err_t ret = this->sensor_driver_->start_stream();
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Échec start sensor");
      return false;
    }
    delay(100);
  }

#ifdef USE_ESP32_VARIANT_ESP32P4
  // Démarrer CSI
  esp_err_t ret = esp_cam_ctlr_start(this->csi_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Échec start CSI");
    return false;
  }
#endif

  this->streaming_ = true;
  ESP_LOGI(TAG, "✅ Streaming actif");
  return true;
}

bool MIPICameraComponent::stop_streaming() {
  if (!this->streaming_) {
    return true;
  }

#ifdef USE_ESP32_VARIANT_ESP32P4
  esp_cam_ctlr_stop(this->csi_handle_);
#endif

  if (this->sensor_driver_) {
    this->sensor_driver_->stop_stream();
  }

  this->streaming_ = false;
  ESP_LOGI(TAG, "Streaming arrêté");
  return true;
}

// ==================================================
// FRAME CAPTURE avec PPA
// ==================================================

bool MIPICameraComponent::capture_frame() {
  if (!this->streaming_) {
    return false;
  }

  if (!this->display_buffer_ready_.load(std::memory_order_acquire)) {
    return false;
  }

  // Swap atomique
  uint8_t ready_idx = this->ready_buffer_index_.load(std::memory_order_acquire);
  uint8_t* source_buffer = this->frame_buffers_[ready_idx];
  this->display_buffer_ready_.store(false, std::memory_order_release);

#ifdef USE_ESP32_VARIANT_ESP32P4
  // ✅ Utiliser PPA pour mirror/flip au lieu du capteur
  if (this->use_ppa_mirror_ && this->ppa_handle_ && this->sensor_driver_) {
    const auto& info = this->sensor_driver_->get_info();
    
    ppa_srm_oper_config_t srm_config = {
      .in = {
        .buffer = source_buffer,
        .pic_w = info.width,
        .pic_h = info.height,
        .block_w = info.width,
        .block_h = info.height,
        .block_offset_x = 0,
        .block_offset_y = 0,
        .srm_cm = PPA_SRM_COLOR_MODE_RGB565
      },
      .out = {
        .buffer = this->ppa_output_buffer_,
        .buffer_size = this->frame_buffer_size_,
        .pic_w = info.width,
        .pic_h = info.height,
        .block_offset_x = 0,
        .block_offset_y = 0,
        .srm_cm = PPA_SRM_COLOR_MODE_RGB565
      },
      .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
      .scale_x = 1.0f,
      .scale_y = 1.0f,
      .mirror_x = this->mirror_enabled_,  // ✅ Mirror via PPA
      .mirror_y = this->flip_enabled_,    // ✅ Flip via PPA
      .rgb_swap = false,
      .byte_swap = false,
      .mode = PPA_TRANS_MODE_BLOCKING
    };

    esp_err_t ret = ppa_do_scale_rotate_mirror(this->ppa_handle_, &srm_config);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "PPA mirror échoué: 0x%x", ret);
      this->display_buffer_ = source_buffer;  // Fallback
    } else {
      this->display_buffer_ = this->ppa_output_buffer_;
    }
  } else {
    this->display_buffer_ = source_buffer;
  }
#else
  this->display_buffer_ = source_buffer;
#endif

  // Appliquer WB software si nécessaire
  if (this->sensor_driver_) {
    uint32_t frame_num = this->frame_number_.load();
    
    if (frame_num % white_balance::APPLY_EVERY_N_FRAMES == 0) {
      const auto& info = this->sensor_driver_->get_info();
      
      if (debug::LOG_WB_APPLICATION && frame_num % 100 == 0) {
        ESP_LOGD(TAG, "WB gains: R=%.2f G=%.2f B=%.2f",
                 white_balance::RED_GAIN,
                 white_balance::GREEN_GAIN,
                 white_balance::BLUE_GAIN);
      }
      
      this->color_correction_.apply_software_wb_center(
        this->display_buffer_,
        info.width,
        info.height,
        0.6f
      );
    }
  }

  return true;
}

// ==================================================
// ISR CALLBACKS
// ==================================================

#ifdef USE_ESP32_VARIANT_ESP32P4
bool IRAM_ATTR MIPICameraComponent::on_csi_new_frame_(
  esp_cam_ctlr_handle_t handle,
  esp_cam_ctlr_trans_t *trans,
  void *user_data
) {
  MIPICameraComponent *cam = (MIPICameraComponent*)user_data;

  uint8_t idx = cam->capture_buffer_index_.load(std::memory_order_relaxed);
  trans->buffer = cam->frame_buffers_[idx];
  trans->buflen = cam->frame_buffer_size_;

  return false;
}

bool IRAM_ATTR MIPICameraComponent::on_csi_frame_done_(
  esp_cam_ctlr_handle_t handle,
  esp_cam_ctlr_trans_t *trans,
  void *user_data
) {
  MIPICameraComponent *cam = (MIPICameraComponent*)user_data;

  if (trans->received_size > 0) {
    // Rotation atomique des buffers
    uint8_t old_capture = cam->capture_buffer_index_.load(std::memory_order_acquire);
    uint8_t old_ready = cam->ready_buffer_index_.load(std::memory_order_acquire);

    cam->ready_buffer_index_.store(old_capture, std::memory_order_release);
    cam->capture_buffer_index_.store(old_ready, std::memory_order_release);

    cam->display_buffer_ready_.store(true, std::memory_order_release);
    cam->frame_number_.fetch_add(1, std::memory_order_relaxed);
    cam->total_frames_received_++;
  }

  return false;
}
#endif

// ==================================================
// AUTO EXPOSURE
// ==================================================

void MIPICameraComponent::ae_task_(void* param) {
  MIPICameraComponent* cam = (MIPICameraComponent*)param;
  AECommand cmd;

  ESP_LOGI(TAG, "Tâche AE démarrée");

  while (true) {
    if (xQueueReceive(cam->ae_command_queue_, &cmd, portMAX_DELAY) == pdTRUE) {
      if (cam->sensor_driver_) {
        cam->sensor_driver_->set_exposure(cmd.exposure);
        vTaskDelay(pdMS_TO_TICKS(5));
        cam->sensor_driver_->set_gain(cmd.gain);
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

void MIPICameraComponent::update_auto_exposure_() {
  if (!this->auto_exposure_enabled_ || !this->sensor_driver_) {
    return;
  }

  uint32_t now = millis();
  if (now - this->last_ae_update_ < auto_exposure::UPDATE_INTERVAL_MS) {
    return;
  }
  this->last_ae_update_ = now;

  uint32_t avg_brightness = this->calculate_brightness_();
  int32_t error = (int32_t)auto_exposure::TARGET_BRIGHTNESS - (int32_t)avg_brightness;

  if (abs(error) > auto_exposure::ADJUSTMENT_THRESHOLD) {
    const auto& info = this->sensor_driver_->get_info();
    bool changed = false;

    if (error > 0) {  // Trop sombre
      if (this->current_exposure_ < info.max_exposure) {
        this->current_exposure_ += auto_exposure::EXPOSURE_STEP;
        changed = true;
      } else if (this->current_gain_index_ < info.max_gain_index) {
        this->current_gain_index_ += auto_exposure::GAIN_STEP;
        changed = true;
      }
    } else {  // Trop lumineux
      if (this->current_exposure_ > info.min_exposure) {
        this->current_exposure_ -= auto_exposure::EXPOSURE_STEP;
        changed = true;
      } else if (this->current_gain_index_ > info.min_gain_index) {
        this->current_gain_index_ -= auto_exposure::GAIN_STEP;
        changed = true;
      }
    }

    if (changed && this->ae_command_queue_) {
      AECommand cmd = {this->current_exposure_, this->current_gain_index_};
      xQueueSend(this->ae_command_queue_, &cmd, 0);
    }
  }
}

uint32_t MIPICameraComponent::calculate_brightness_() {
  if (!this->display_buffer_ || !this->sensor_driver_) {
    return 128;
  }

  const auto& info = this->sensor_driver_->get_info();
  uint32_t center_offset = (info.height / 2) * info.width * 2 + (info.width / 2) * 2;
  
  uint32_t sum = 0;
  uint32_t count = 0;

  for (size_t i = 0; i < optimization::AE_SAMPLE_COUNT; i++) {
    uint32_t offset = center_offset + (i * 400);
    if (offset + 1 < this->frame_buffer_size_) {
      uint16_t pixel = (this->display_buffer_[offset + 1] << 8) | this->display_buffer_[offset];

      uint8_t r = (pixel >> 11) & 0x1F;
      uint8_t g = (pixel >> 5) & 0x3F;
      uint8_t b = pixel & 0x1F;

      sum += (r * 8 * 299 + g * 4 * 587 + b * 8 * 114) / 1000;
      count++;
    }
  }

  return count > 0 ? (sum / count) : 128;
}

// ==================================================
// CLEANUP
// ==================================================

void MIPICameraComponent::cleanup_() {
#ifdef USE_ESP32_VARIANT_ESP32P4
  if (this->ppa_handle_) {
    ppa_unregister_client(this->ppa_handle_);
    this->ppa_handle_ = nullptr;
    ESP_LOGI(TAG, "PPA cleanup OK");
  }
  
  if (this->ppa_output_buffer_) {
    heap_caps_free(this->ppa_output_buffer_);
    this->ppa_output_buffer_ = nullptr;
  }
#endif
}

// ==================================================
// LOOP & CONFIG
// ==================================================

void MIPICameraComponent::loop() {
  if (this->streaming_) {
    this->update_auto_exposure_();

    if (debug::LOG_FPS) {
      uint32_t now = millis();
      if (now - this->last_frame_log_time_ >= optimization::FPS_LOG_INTERVAL_MS) {
        float fps = this->total_frames_received_ * 1000.0f / optimization::FPS_LOG_INTERVAL_MS;

        ESP_LOGI(TAG, "📸 FPS: %.1f | Frames: %u | Exp: 0x%04X | Gain: %u",
                 fps, this->frame_number_.load(),
                 this->current_exposure_, this->current_gain_index_);

        this->total_frames_received_ = 0;
        this->last_frame_log_time_ = now;
      }
    }
  }
}

void MIPICameraComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "MIPI Camera:");
  
  if (!this->name_.empty()) {
    ESP_LOGCONFIG(TAG, "  Name: %s", this->name_.c_str());
  }
  
  if (this->sensor_driver_) {
    const auto& info = this->sensor_driver_->get_info();
    ESP_LOGCONFIG(TAG, "  Sensor: %s (%s)", info.name, info.manufacturer);
    ESP_LOGCONFIG(TAG, "  Resolution: %dx%d @ %d fps", info.width, info.height, info.fps);
  }
  
  ESP_LOGCONFIG(TAG, "  Config Resolution: %s", this->resolution_.c_str());
  ESP_LOGCONFIG(TAG, "  Pixel Format: %s", this->pixel_format_.c_str());
  ESP_LOGCONFIG(TAG, "  Target Framerate: %d fps", this->framerate_);
  ESP_LOGCONFIG(TAG, "  JPEG Quality: %d", this->jpeg_quality_);
  ESP_LOGCONFIG(TAG, "  Triple buffering: %d buffers", NUM_BUFFERS);
  ESP_LOGCONFIG(TAG, "  CCM: %s", 
                this->color_correction_.is_hardware_ccm_enabled() ? "Hardware" : "Software");
  ESP_LOGCONFIG(TAG, "  Auto Exposure: %s", 
                this->auto_exposure_enabled_ ? "ON" : "OFF");
  ESP_LOGCONFIG(TAG, "  Mirror (PPA): %s", this->mirror_enabled_ ? "ON" : "OFF");
  ESP_LOGCONFIG(TAG, "  Flip (PPA): %s", this->flip_enabled_ ? "ON" : "OFF");
  ESP_LOGCONFIG(TAG, "  PPA: %s", this->use_ppa_mirror_ ? "Enabled" : "Disabled");
  ESP_LOGCONFIG(TAG, "  Initialized: %s", this->initialized_ ? "YES" : "NO");
}

// ==================================================
// PUBLIC API
// ==================================================

uint16_t MIPICameraComponent::get_image_width() const {
  if (this->sensor_driver_ == nullptr) {
    return 0;
  }
  return this->sensor_driver_->get_info().width;
}

uint16_t MIPICameraComponent::get_image_height() const {
  if (this->sensor_driver_ == nullptr) {
    return 0;
  }
  return this->sensor_driver_->get_info().height;
}

void MIPICameraComponent::set_auto_exposure(bool enabled) {
  this->auto_exposure_enabled_ = enabled;
  ESP_LOGI(TAG, "Auto Exposure: %s", enabled ? "ON" : "OFF");
}

void MIPICameraComponent::set_manual_exposure(uint16_t exposure) {
  this->current_exposure_ = exposure;
  if (this->sensor_driver_ && this->ae_command_queue_) {
    AECommand cmd = {exposure, this->current_gain_index_};
    xQueueSend(this->ae_command_queue_, &cmd, 0);
    ESP_LOGI(TAG, "Exposition manuelle: 0x%04X", exposure);
  }
}

void MIPICameraComponent::set_manual_gain(uint8_t gain_index) {
  this->current_gain_index_ = gain_index;
  if (this->sensor_driver_ && this->ae_command_queue_) {
    AECommand cmd = {this->current_exposure_, gain_index};
    xQueueSend(this->ae_command_queue_, &cmd, 0);
    ESP_LOGI(TAG, "Gain manuel: %u", gain_index);
  }
}

void MIPICameraComponent::set_brightness_level(uint8_t level) {
  if (!this->sensor_driver_) {
    return;
  }
  
  if (level > 10) level = 10;
  
  const auto& info = this->sensor_driver_->get_info();
  uint16_t exposure_range = info.max_exposure - info.min_exposure;
  uint16_t exposure = info.min_exposure + (exposure_range * level / 10);
  
  uint8_t gain = info.min_gain_index + ((info.max_gain_index - info.min_gain_index) * level / 10);
  
  set_manual_exposure(exposure);
  vTaskDelay(pdMS_TO_TICKS(50));
  set_manual_gain(gain);
}

void MIPICameraComponent::set_color_profile(ColorCorrection::CCMProfile profile) {
  this->color_correction_.set_ccm_profile(profile);
  ESP_LOGI(TAG, "Profil couleur changé");
}

void MIPICameraComponent::set_flip(bool enable) {
  this->flip_enabled_ = enable;
  ESP_LOGI(TAG, "Flip (PPA): %s", enable ? "ON" : "OFF");
}

void MIPICameraComponent::set_mirror(bool enable) {
  this->mirror_enabled_ = enable;
  ESP_LOGI(TAG, "Mirror (PPA): %s", enable ? "ON" : "OFF");
}

}  // namespace mipi_camera
}  // namespace esphome
