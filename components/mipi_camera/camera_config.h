#pragma once

/**
 * ============================================
 * CONFIGURATION GLOBALE CAMÉRA MIPI
 * ============================================
 * Fichier centralisé pour tous les paramètres
 */

namespace esphome {
namespace mipi_camera {

// ============================================
// PERFORMANCE & BUFFERING
// ============================================

namespace performance {
  constexpr size_t NUM_BUFFERS = 3;           // Triple buffering
  constexpr uint32_t ISP_CLOCK_HZ = 120000000; // 120 MHz
  constexpr size_t BUFFER_ALIGNMENT = 64;     // Alignement PSRAM
}

// ============================================
// AUTO EXPOSURE (AE)
// ============================================

namespace auto_exposure {
  constexpr bool ENABLED_BY_DEFAULT = true;
  constexpr uint8_t TARGET_BRIGHTNESS = 128;
  constexpr uint32_t UPDATE_INTERVAL_MS = 800;
  constexpr uint8_t ADJUSTMENT_THRESHOLD = 20;
  constexpr uint16_t EXPOSURE_STEP = 0x100;
  constexpr uint8_t GAIN_STEP = 5;
}

// ============================================
// COLOR CORRECTION MATRIX (CCM)
// ============================================

namespace color_correction {
  // CCM pour SC202CS - Lumière intérieure standard
  constexpr float CCM_INDOOR[9] = {
    1.65f, -0.45f, -0.20f,  // Rouge: augmenté, réduit vert
    -0.25f,  1.35f, -0.10f,  // Vert: réduit
    -0.15f, -0.50f,  1.65f   // Bleu: augmenté, réduit vert
  };
  
  // CCM pour lumière LED froide
  constexpr float CCM_LED_COOL[9] = {
    1.55f, -0.35f, -0.20f,
    -0.20f,  1.30f, -0.10f,
    -0.10f, -0.45f,  1.55f
  };
  
  // CCM pour lumière naturelle/extérieur
  constexpr float CCM_OUTDOOR[9] = {
    1.35f, -0.25f, -0.10f,
    -0.15f,  1.25f, -0.10f,
    -0.10f, -0.30f,  1.40f
  };
  
  constexpr bool ENABLE_SATURATION = true;
}

// ============================================
// WHITE BALANCE (WB) - Software fallback
// ============================================

namespace white_balance {
  // ✅ CORRECTION POUR TEINTE VERTE
  // Une feuille blanche apparaît verte = trop de vert, pas assez de rouge/bleu
  
  // 🎯 Option 1: RÉDUIRE LE VERT (recommandé)
  constexpr float RED_GAIN = 1.3f;     // Augmenter rouge
  constexpr float GREEN_GAIN = 1.0f;   // RÉDUIRE vert (était 1.0)
  constexpr float BLUE_GAIN = 1.4f;    // Augmenter bleu
  
  // 🎯 Option 2: Si Option 1 trop rose/magenta
  // Décommentez ces lignes et commentez Option 1:
  // constexpr float RED_GAIN = 1.2f;
  // constexpr float GREEN_GAIN = 0.8f;
  // constexpr float BLUE_GAIN = 1.3f;
  
  // 🎯 Option 3: Si Option 1 pas assez fort
  // constexpr float RED_GAIN = 1.5f;
  // constexpr float GREEN_GAIN = 0.6f;
  // constexpr float BLUE_GAIN = 1.5f;
  
  // Fréquence d'application WB software
  constexpr uint8_t APPLY_EVERY_N_FRAMES = 1;  // Chaque frame
}

// ============================================
// OPTIMISATIONS FPS
// ============================================

namespace optimization {
  // Désactiver AE en mode plein écran pour max FPS
  constexpr bool DISABLE_AE_FULLSCREEN = false;
  
  // Échantillonnage AE réduit (nombre de pixels analysés)
  constexpr size_t AE_SAMPLE_COUNT = 50;
  
  // Intervalle de stats FPS (ms)
  constexpr uint32_t FPS_LOG_INTERVAL_MS = 3000;
}

// ============================================
// DEBUG & LOGS
// ============================================

namespace debug {
  constexpr bool VERBOSE_INIT = true;
  constexpr bool LOG_FPS = true;
  constexpr bool LOG_AE_CHANGES = false;
  constexpr bool LOG_WB_APPLICATION = true;  // ← Nouveau: log WB
}

}  // namespace mipi_camera
}  // namespace esphome
