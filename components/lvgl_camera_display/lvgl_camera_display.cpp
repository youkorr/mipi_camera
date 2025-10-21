#include "lvgl_camera_display.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace lvgl_camera_display {

static const char *const TAG = "lvgl_camera_display";

void LVGLCameraDisplay::setup() {
  ESP_LOGCONFIG(TAG, "🎥 LVGL Camera Display (Zero-Copy Mode)");

  // ✅ CRITIQUE: Vérifier que la caméra existe
  if (this->camera_ == nullptr) {
    ESP_LOGE(TAG, "❌ Camera not configured");
    this->mark_failed();
    return;
  }

  // ✅ CRITIQUE: Ne PAS accéder à la caméra pendant setup()
  // La caméra n'est peut-être pas encore initialisée
  ESP_LOGI(TAG, "✅ Display initialized (waiting for camera)");
}

void LVGLCameraDisplay::loop() {
  // ✅ CRITIQUE: Vérifications de sécurité complètes
  if (this->camera_ == nullptr) {
    return;
  }

  // Vérifier que la caméra est initialisée
  if (!this->camera_->is_initialized()) {
    return;
  }

  // Vérifier que le streaming est actif
  if (!this->camera_->is_streaming()) {
    return;
  }
  
  // ✅ Capturer la frame (swap de buffer atomique)
  if (this->camera_->capture_frame()) {
    this->update_canvas_();
    this->frame_count_++;

    // Logger FPS réel toutes les 100 frames
    if (this->frame_count_ % 100 == 0) {
      uint32_t now_time = millis();

      if (this->last_fps_time_ > 0) {
        float elapsed = (now_time - this->last_fps_time_) / 1000.0f;
        float fps = 100.0f / elapsed;
        ESP_LOGI(TAG, "🎞️ Display FPS: %.2f | %u frames total", fps, this->frame_count_);
      }
      this->last_fps_time_ = now_time;
    }
  }
}

void LVGLCameraDisplay::dump_config() {
  ESP_LOGCONFIG(TAG, "LVGL Camera Display:");
  ESP_LOGCONFIG(TAG, "  Mode: Zero-copy (event-driven)");
  ESP_LOGCONFIG(TAG, "  Canvas: %s", this->canvas_obj_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Camera: %s", this->camera_ ? "YES" : "NO");
}

void LVGLCameraDisplay::update_canvas_() {
  // ✅ CRITIQUE: Vérifications complètes
  if (this->camera_ == nullptr) {
    if (!this->canvas_warning_shown_) {
      ESP_LOGW(TAG, "❌ Camera null");
      this->canvas_warning_shown_ = true;
    }
    return;
  }

  if (this->canvas_obj_ == nullptr) {
    if (!this->canvas_warning_shown_) {
      ESP_LOGW(TAG, "❌ Canvas null");
      this->canvas_warning_shown_ = true;
    }
    return;
  }

  // ✅ Vérifier que la caméra est initialisée avant d'accéder aux données
  if (!this->camera_->is_initialized()) {
    return;
  }

  uint8_t* img_data = this->camera_->get_image_data();
  
  // ✅ Vérifier que les données existent
  if (img_data == nullptr) {
    return;
  }

  uint16_t width = this->camera_->get_image_width();
  uint16_t height = this->camera_->get_image_height();

  // ✅ Vérifier que les dimensions sont valides
  if (width == 0 || height == 0) {
    return;
  }

  if (this->first_update_) {
    ESP_LOGI(TAG, "🖼️  First canvas update:");
    ESP_LOGI(TAG, "   Dimensions: %ux%u", width, height);
    ESP_LOGI(TAG, "   Buffer: %p", img_data);
    ESP_LOGI(TAG, "   Buffer size: %u bytes", width * height * 2);
    this->first_update_ = false;
  }

  // ✅ CRITIQUE: Utiliser lv_canvas_set_buffer pour pointer vers le buffer caméra
  // Cela évite toute copie de mémoire (zero-copy)
  // Note: Le format peut être LV_IMG_CF_TRUE_COLOR (LVGL v8) ou LV_COLOR_FORMAT_RGB565 (LVGL v9)
  if (this->last_buffer_ptr_ != img_data) {
    #if LV_VERSION_CHECK(9, 0, 0)
      // LVGL v9+
      lv_canvas_set_buffer(
        this->canvas_obj_, 
        img_data, 
        width, 
        height, 
        LV_COLOR_FORMAT_RGB565
      );
    #else
      // LVGL v8
      lv_canvas_set_buffer(
        this->canvas_obj_, 
        img_data, 
        width, 
        height, 
        LV_IMG_CF_TRUE_COLOR
      );
    #endif
    
    this->last_buffer_ptr_ = img_data;
    
    if (this->frame_count_ % 100 == 0) {
      ESP_LOGD(TAG, "Buffer updated: %p", img_data);
    }
  }
  
  // ✅ Invalider pour forcer le rafraîchissement
  lv_obj_invalidate(this->canvas_obj_);
}

void LVGLCameraDisplay::configure_canvas(lv_obj_t *canvas) { 
  this->canvas_obj_ = canvas;
  ESP_LOGI(TAG, "🎨 Canvas configured: %p", canvas);

  if (canvas != nullptr) {
    lv_coord_t w = lv_obj_get_width(canvas);
    lv_coord_t h = lv_obj_get_height(canvas);
    ESP_LOGI(TAG, "   Canvas size: %dx%d", w, h);
    
    // Optimisations canvas
    lv_obj_clear_flag(canvas, LV_OBJ_FLAG_SCROLLABLE);
    
    // S'assurer que le canvas est visible
    lv_obj_clear_flag(canvas, LV_OBJ_FLAG_HIDDEN);
  }
}

}  // namespace lvgl_camera_display
}  // namespace esphome
