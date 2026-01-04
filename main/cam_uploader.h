#pragma once

#include "esp_err.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char url[256];
    int interval_sec;
} cam_uploader_config_t;

/** Load config from NVS (or defaults) and create internal locks. */
esp_err_t cam_uploader_init(void);

/** Initialize camera (idempotent). */
esp_err_t cam_uploader_camera_init(void);

/** Start background uploader task (safe to call once). */
esp_err_t cam_uploader_start(void);

/** Get current config (thread-safe copy). */
esp_err_t cam_uploader_get_config(cam_uploader_config_t *out_cfg);

/** Set config (persist to NVS + notify task). */
esp_err_t cam_uploader_set_config(const cam_uploader_config_t *cfg);

/** Notify uploader about WiFi connectivity changes. */
void cam_uploader_set_wifi_connected(bool connected);

#ifdef __cplusplus
}
#endif
