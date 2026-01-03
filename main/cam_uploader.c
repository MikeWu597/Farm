#include "cam_uploader.h"

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_camera.h"
#include "sdkconfig.h"

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif

// If the user doesn't select a camera model at build time,
// default to the most common ESP32-CAM module.
#if !defined(CAMERA_MODEL_WROVER_KIT) && !defined(CAMERA_MODEL_ESP_EYE) && \
    !defined(CAMERA_MODEL_ESP32S3_EYE) && !defined(CAMERA_MODEL_M5STACK_PSRAM) && \
    !defined(CAMERA_MODEL_M5STACK_V2_PSRAM) && !defined(CAMERA_MODEL_M5STACK_WIDE) && \
    !defined(CAMERA_MODEL_M5STACK_ESP32CAM) && !defined(CAMERA_MODEL_M5STACK_UNITCAM) && \
    !defined(CAMERA_MODEL_AI_THINKER) && !defined(CAMERA_MODEL_TTGO_T_JOURNAL) && \
    !defined(CAMERA_MODEL_XIAO_ESP32S3) && !defined(CAMERA_MODEL_ESP32_CAM_BOARD) && \
    !defined(CAMERA_MODEL_ESP32S2_CAM_BOARD) && !defined(CAMERA_MODEL_ESP32S3_CAM_LCD) && \
    !defined(CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3) && !defined(CAMERA_MODEL_DFRobot_Romeo_ESP32S3)
#define CAMERA_MODEL_AI_THINKER
#endif

#include "camera_pins.h"

static const char *TAG = "cam_uploader";

#define NVS_NS "uploader"
#define NVS_KEY_URL "url"
#define NVS_KEY_INTERVAL "interval"

static SemaphoreHandle_t s_lock;
static cam_uploader_config_t s_cfg;
static TaskHandle_t s_task;
static bool s_wifi_connected;
static bool s_camera_inited;

static void cfg_set_defaults(cam_uploader_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->interval_sec = 60;
    cfg->url[0] = '\0';
}

static esp_err_t nvs_load_cfg(cam_uploader_config_t *cfg)
{
    cfg_set_defaults(cfg);

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return ESP_OK;
    }

    size_t url_len = sizeof(cfg->url);
    err = nvs_get_str(h, NVS_KEY_URL, cfg->url, &url_len);
    if (err != ESP_OK) {
        cfg->url[0] = '\0';
    }

    int32_t interval = 0;
    err = nvs_get_i32(h, NVS_KEY_INTERVAL, &interval);
    if (err == ESP_OK && interval > 0) {
        cfg->interval_sec = (int)interval;
    }

    nvs_close(h);
    return ESP_OK;
}

static esp_err_t nvs_save_cfg(const cam_uploader_config_t *cfg)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_str(h, NVS_KEY_URL, cfg->url);
    if (err == ESP_OK) {
        err = nvs_set_i32(h, NVS_KEY_INTERVAL, (int32_t)cfg->interval_sec);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t cam_uploader_init(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        if (!s_lock) {
            return ESP_ERR_NO_MEM;
        }
    }

    cam_uploader_config_t cfg;
    ESP_RETURN_ON_ERROR(nvs_load_cfg(&cfg), TAG, "nvs_load_cfg failed");

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg = cfg;
    xSemaphoreGive(s_lock);

    return ESP_OK;
}

esp_err_t cam_uploader_get_config(cam_uploader_config_t *out_cfg)
{
    if (!out_cfg || !s_lock) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out_cfg = s_cfg;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t cam_uploader_set_config(const cam_uploader_config_t *cfg)
{
    if (!cfg || !s_lock) {
        return ESP_ERR_INVALID_ARG;
    }

    cam_uploader_config_t cleaned = *cfg;
    cleaned.url[sizeof(cleaned.url) - 1] = '\0';
    if (cleaned.interval_sec < 1) {
        cleaned.interval_sec = 1;
    }

    ESP_RETURN_ON_ERROR(nvs_save_cfg(&cleaned), TAG, "nvs_save_cfg failed");

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg = cleaned;
    xSemaphoreGive(s_lock);

    if (s_task) {
        xTaskNotifyGive(s_task);
    }

    return ESP_OK;
}

void cam_uploader_set_wifi_connected(bool connected)
{
    s_wifi_connected = connected;
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

esp_err_t cam_uploader_camera_init(void)
{
    if (s_camera_inited) {
        return ESP_OK;
    }

    camera_config_t config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pin_d0 = Y2_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM,
        .pin_xclk = XCLK_GPIO_NUM,
        .pin_pclk = PCLK_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href = HREF_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,
        .pin_pwdn = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .xclk_freq_hz = 20000000,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location = CAMERA_FB_IN_DRAM,
    };

#if CONFIG_SPIRAM
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
#endif

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: 0x%x", err);
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, FRAMESIZE_QVGA);
    }

    s_camera_inited = true;
    ESP_LOGI(TAG, "camera initialized");
    return ESP_OK;
}

static esp_err_t http_post_jpeg(const char *url, const uint8_t *buf, size_t len)
{
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 15000,
    };

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
    config.crt_bundle_attach = esp_crt_bundle_attach;
#endif

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_set_header(client, "Content-Type", "image/jpeg");
    esp_http_client_set_post_field(client, (const char *)buf, (int)len);

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "POST failed: %s", esp_err_to_name(err));
        return err;
    }

    if (status < 200 || status >= 300) {
        ESP_LOGW(TAG, "POST http status=%d", status);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void uploader_task(void *arg)
{
    (void)arg;

    for (;;) {
        // Wait until WiFi is connected.
        while (!s_wifi_connected) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        cam_uploader_config_t cfg;
        cam_uploader_get_config(&cfg);

        // Disabled until URL is set.
        if (cfg.url[0] == '\0') {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }

        if (!s_camera_inited) {
            esp_err_t cam_err = cam_uploader_camera_init();
            if (cam_err != ESP_OK) {
                // Wait a bit (or config change) before retry.
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000));
                continue;
            }
        }

        int64_t t0 = esp_timer_get_time();
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "camera capture failed");
        } else if (fb->format != PIXFORMAT_JPEG) {
            ESP_LOGW(TAG, "frame format not JPEG (%d)", fb->format);
            esp_camera_fb_return(fb);
        } else {
            esp_err_t post_err = http_post_jpeg(cfg.url, fb->buf, fb->len);
            esp_camera_fb_return(fb);

            int64_t dt_ms = (esp_timer_get_time() - t0) / 1000;
            if (post_err == ESP_OK) {
                ESP_LOGI(TAG, "uploaded %u bytes in %lld ms", (unsigned)fb->len, (long long)dt_ms);
            }
        }

        // Sleep until next interval, but wake early if config changes or WiFi state changes.
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(cfg.interval_sec * 1000));
    }
}

esp_err_t cam_uploader_start(void)
{
    if (s_task) {
        return ESP_OK;
    }
    if (!s_lock) {
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ok = xTaskCreate(uploader_task, "cam_uploader", 8192, NULL, 5, &s_task);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
