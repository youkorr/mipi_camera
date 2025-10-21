#pragma once

#include "esphome/core/component.h"
#include "esphome/components/lvgl/lvgl_esphome.h"
#include "../mipi_camera/mipi_camera.h"

namespace esphome {
namespace lvgl_camera_display {

/**
 * Composant pour afficher le flux caméra MIPI dans un canvas LVGL
 * Mode zero-copy pour performance maximale
 */
class LVGLCameraDisplay : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Configuration
  void set_camera(mipi_camera::MIPICameraComponent *camera) { 
    this->camera_ = camera; 
  }
  
  void set_canvas_id(const std::string &canvas_id) { 
    this->canvas_id_ = canvas_id; 
  }
  
  void set_update_interval(uint32_t interval_ms) { 
    // Ignoré en mode event-driven
    this->update_interval_ = interval_ms; 
  }

  // Configuration du canvas LVGL
  void configure_canvas(lv_obj_t *canvas);

  // Priorité d'initialisation : après la caméra
  float get_setup_priority() const override { 
    return setup_priority::LATE - 1.0f; 
  }

 protected:
  // Composants liés
  mipi_camera::MIPICameraComponent *camera_{nullptr};
  lv_obj_t *canvas_obj_{nullptr};
  std::string canvas_id_{};

  // Configuration (non utilisée en mode event-driven)
  uint32_t update_interval_{33};
  uint32_t last_update_{0};

  // Statistiques
  uint32_t frame_count_{0};
  uint32_t last_fps_time_{0};
  
  // État
  bool first_update_{true};
  bool canvas_warning_shown_{false};
  
  // Suivi du buffer pour éviter les appels inutiles
  uint8_t* last_buffer_ptr_{nullptr};

  // Méthode interne
  void update_canvas_();
};

}  // namespace lvgl_camera_display
}  // namespace esphome
