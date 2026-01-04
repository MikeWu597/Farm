#include "cam_uploader.h"

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_camera.h"
#include "sdkconfig.h"

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif

// If the user doesn't select a camera model at build time,
// pick a sensible default per target.
#if !defined(CAMERA_MODEL_WROVER_KIT) && !defined(CAMERA_MODEL_ESP_EYE) && \
    !defined(CAMERA_MODEL_ESP32S3_EYE) && !defined(CAMERA_MODEL_M5STACK_PSRAM) && \
    !defined(CAMERA_MODEL_M5STACK_V2_PSRAM) && !defined(CAMERA_MODEL_M5STACK_WIDE) && \
    !defined(CAMERA_MODEL_M5STACK_ESP32CAM) && !defined(CAMERA_MODEL_M5STACK_UNITCAM) && \
    !defined(CAMERA_MODEL_AI_THINKER) && !defined(CAMERA_MODEL_TTGO_T_JOURNAL) && \
    !defined(CAMERA_MODEL_XIAO_ESP32S3) && !defined(CAMERA_MODEL_ESP32_CAM_BOARD) && \
    !defined(CAMERA_MODEL_ESP32S2_CAM_BOARD) && !defined(CAMERA_MODEL_ESP32S3_CAM_LCD) && \
    !defined(CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3) && !defined(CAMERA_MODEL_DFRobot_Romeo_ESP32S3)
#if CONFIG_IDF_TARGET_ESP32S3
#define CAMERA_MODEL_ESP32S3_CAM_LCD
#else
#define CAMERA_MODEL_AI_THINKER
#endif
#endif

#include "camera_pins.h"

static const char *TAG = "cam_uploader";

#define NVS_NS "uploader"
#define NVS_KEY_URL "url"
#define NVS_KEY_VOLTAGE_URL "vurl"
#define NVS_KEY_INTERVAL "interval"

#define VBAT_ADC_UNIT ADC_UNIT_1
#define VBAT_ADC_CHANNEL ADC_CHANNEL_0
#define VBAT_ADC_ATTEN ADC_ATTEN_DB_12
#define VBAT_DIVIDER_NUM 2
#define VBAT_DIVIDER_DEN 1
#define VBAT_APPROX_FULLSCALE_MV 3300

static SemaphoreHandle_t s_lock;
static cam_uploader_config_t s_cfg;
static TaskHandle_t s_task;
static bool s_wifi_connected;
static bool s_camera_inited;

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }
    return -1;
}

static bool url_percent_decode(const char *src, char *dst, size_t dst_len)
{
    if (!src || !dst || dst_len == 0) {
        return false;
    }

    size_t di = 0;
    for (size_t si = 0; src[si] != '\0'; si++) {
        if (di + 1 >= dst_len) {
            return false;
        }

        if (src[si] == '%' && src[si + 1] && src[si + 2]) {
            int hi = hex_nibble(src[si + 1]);
            int lo = hex_nibble(src[si + 2]);
            if (hi >= 0 && lo >= 0) {
                dst[di++] = (char)((hi << 4) | lo);
                si += 2;
                continue;
            }
        }

        dst[di++] = src[si];
    }

    dst[di] = '\0';
    return true;
}

static void normalize_url_inplace(char *url, size_t url_len)
{
    if (!url || url_len == 0 || url[0] == '\0') {
        return;
    }

    // Decode percent-encoded URLs commonly produced by web forms.
    if (strchr(url, '%')) {
        char tmp[256];
        if (url_len <= sizeof(tmp) && url_percent_decode(url, tmp, sizeof(tmp))) {
            // Only accept if it looks like a real URL after decoding.
            if (strncmp(tmp, "http://", 7) == 0 || strncmp(tmp, "https://", 8) == 0) {
                strncpy(url, tmp, url_len);
                url[url_len - 1] = '\0';
            }
        }
    }
}

typedef struct {
    const char *name;
    int pin_pwdn;
    int pin_reset;
    int pin_xclk;
    int pin_sccb_sda;
    int pin_sccb_scl;
    int pin_d0;
    int pin_d1;
    int pin_d2;
    int pin_d3;
    int pin_d4;
    int pin_d5;
    int pin_d6;
    int pin_d7;
    int pin_vsync;
    int pin_href;
    int pin_pclk;
} cam_model_pins_t;

// Pin values copied from sdk/camera_pins.h. Order is chosen to prioritize ESP32-S3 boards first.
static const cam_model_pins_t s_cam_model_try_list[] = {
#if CONFIG_IDF_TARGET_ESP32S3
    // ESP32-S3 boards
    {
        .name = "ESP32S3_CAM_LCD",
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = 40,
        .pin_sccb_sda = 17,
        .pin_sccb_scl = 18,
        .pin_d0 = 13,
        .pin_d1 = 47,
        .pin_d2 = 14,
        .pin_d3 = 3,
        .pin_d4 = 12,
        .pin_d5 = 42,
        .pin_d6 = 41,
        .pin_d7 = 39,
        .pin_vsync = 21,
        .pin_href = 38,
        .pin_pclk = 11,
    },
    {
        .name = "ESP32S3_EYE",
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = 15,
        .pin_sccb_sda = 4,
        .pin_sccb_scl = 5,
        .pin_d0 = 11,
        .pin_d1 = 9,
        .pin_d2 = 8,
        .pin_d3 = 10,
        .pin_d4 = 12,
        .pin_d5 = 18,
        .pin_d6 = 17,
        .pin_d7 = 16,
        .pin_vsync = 6,
        .pin_href = 7,
        .pin_pclk = 13,
    },
    {
        .name = "XIAO_ESP32S3",
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = 10,
        .pin_sccb_sda = 40,
        .pin_sccb_scl = 39,
        .pin_d0 = 15,
        .pin_d1 = 17,
        .pin_d2 = 18,
        .pin_d3 = 16,
        .pin_d4 = 14,
        .pin_d5 = 12,
        .pin_d6 = 11,
        .pin_d7 = 48,
        .pin_vsync = 38,
        .pin_href = 47,
        .pin_pclk = 13,
    },
    {
        .name = "DFRobot_FireBeetle2_ESP32S3",
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = 45,
        .pin_sccb_sda = 1,
        .pin_sccb_scl = 2,
        .pin_d0 = 39,
        .pin_d1 = 40,
        .pin_d2 = 41,
        .pin_d3 = 4,
        .pin_d4 = 7,
        .pin_d5 = 8,
        .pin_d6 = 46,
        .pin_d7 = 48,
        .pin_vsync = 6,
        .pin_href = 42,
        .pin_pclk = 5,
    },
    {
        .name = "DFRobot_Romeo_ESP32S3",
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = 45,
        .pin_sccb_sda = 1,
        .pin_sccb_scl = 2,
        .pin_d0 = 39,
        .pin_d1 = 40,
        .pin_d2 = 41,
        .pin_d3 = 4,
        .pin_d4 = 7,
        .pin_d5 = 8,
        .pin_d6 = 46,
        .pin_d7 = 48,
        .pin_vsync = 6,
        .pin_href = 42,
        .pin_pclk = 5,
    },
#endif

    // Common ESP32-CAM style boards
    {
        .name = "AI_THINKER",
        .pin_pwdn = 32,
        .pin_reset = -1,
        .pin_xclk = 0,
        .pin_sccb_sda = 26,
        .pin_sccb_scl = 27,
        .pin_d0 = 5,
        .pin_d1 = 18,
        .pin_d2 = 19,
        .pin_d3 = 21,
        .pin_d4 = 36,
        .pin_d5 = 39,
        .pin_d6 = 34,
        .pin_d7 = 35,
        .pin_vsync = 25,
        .pin_href = 23,
        .pin_pclk = 22,
    },
    {
        .name = "ESP_EYE",
        .pin_pwdn = 32,
        .pin_reset = -1,
        .pin_xclk = 0,
        .pin_sccb_sda = 26,
        .pin_sccb_scl = 27,
        .pin_d0 = 5,
        .pin_d1 = 18,
        .pin_d2 = 19,
        .pin_d3 = 21,
        .pin_d4 = 36,
        .pin_d5 = 39,
        .pin_d6 = 34,
        .pin_d7 = 35,
        .pin_vsync = 25,
        .pin_href = 23,
        .pin_pclk = 22,
    },
    {
        .name = "WROVER_KIT",
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = 21,
        .pin_sccb_sda = 26,
        .pin_sccb_scl = 27,
        .pin_d0 = 4,
        .pin_d1 = 5,
        .pin_d2 = 18,
        .pin_d3 = 19,
        .pin_d4 = 36,
        .pin_d5 = 39,
        .pin_d6 = 34,
        .pin_d7 = 35,
        .pin_vsync = 25,
        .pin_href = 23,
        .pin_pclk = 22,
    },
    {
        .name = "TTGO_T_JOURNAL",
        .pin_pwdn = 0,
        .pin_reset = 15,
        .pin_xclk = 27,
        .pin_sccb_sda = 25,
        .pin_sccb_scl = 23,
        .pin_d0 = 17,
        .pin_d1 = 35,
        .pin_d2 = 34,
        .pin_d3 = 5,
        .pin_d4 = 39,
        .pin_d5 = 18,
        .pin_d6 = 36,
        .pin_d7 = 19,
        .pin_vsync = 22,
        .pin_href = 26,
        .pin_pclk = 21,
    },
    {
        .name = "M5STACK_PSRAM",
        .pin_pwdn = -1,
        .pin_reset = 15,
        .pin_xclk = 27,
        .pin_sccb_sda = 25,
        .pin_sccb_scl = 23,
        .pin_d0 = 32,
        .pin_d1 = 35,
        .pin_d2 = 34,
        .pin_d3 = 5,
        .pin_d4 = 39,
        .pin_d5 = 18,
        .pin_d6 = 36,
        .pin_d7 = 19,
        .pin_vsync = 22,
        .pin_href = 26,
        .pin_pclk = 21,
    },
    {
        .name = "M5STACK_V2_PSRAM",
        .pin_pwdn = -1,
        .pin_reset = 15,
        .pin_xclk = 27,
        .pin_sccb_sda = 22,
        .pin_sccb_scl = 23,
        .pin_d0 = 32,
        .pin_d1 = 35,
        .pin_d2 = 34,
        .pin_d3 = 5,
        .pin_d4 = 39,
        .pin_d5 = 18,
        .pin_d6 = 36,
        .pin_d7 = 19,
        .pin_vsync = 25,
        .pin_href = 26,
        .pin_pclk = 21,
    },
    {
        .name = "M5STACK_WIDE",
        .pin_pwdn = -1,
        .pin_reset = 15,
        .pin_xclk = 27,
        .pin_sccb_sda = 22,
        .pin_sccb_scl = 23,
        .pin_d0 = 32,
        .pin_d1 = 35,
        .pin_d2 = 34,
        .pin_d3 = 5,
        .pin_d4 = 39,
        .pin_d5 = 18,
        .pin_d6 = 36,
        .pin_d7 = 19,
        .pin_vsync = 25,
        .pin_href = 26,
        .pin_pclk = 21,
    },
    {
        .name = "M5STACK_ESP32CAM",
        .pin_pwdn = -1,
        .pin_reset = 15,
        .pin_xclk = 27,
        .pin_sccb_sda = 25,
        .pin_sccb_scl = 23,
        .pin_d0 = 17,
        .pin_d1 = 35,
        .pin_d2 = 34,
        .pin_d3 = 5,
        .pin_d4 = 39,
        .pin_d5 = 18,
        .pin_d6 = 36,
        .pin_d7 = 19,
        .pin_vsync = 22,
        .pin_href = 26,
        .pin_pclk = 21,
    },
    {
        .name = "M5STACK_UNITCAM",
        .pin_pwdn = -1,
        .pin_reset = 15,
        .pin_xclk = 27,
        .pin_sccb_sda = 25,
        .pin_sccb_scl = 23,
        .pin_d0 = 32,
        .pin_d1 = 35,
        .pin_d2 = 34,
        .pin_d3 = 5,
        .pin_d4 = 39,
        .pin_d5 = 18,
        .pin_d6 = 36,
        .pin_d7 = 19,
        .pin_vsync = 22,
        .pin_href = 26,
        .pin_pclk = 21,
    },
    // ESP32-CAM-BOARD and ESP32S2_CAM_BOARD are omitted here (they are specific wiring variants)
};

static bool gpio_ok_in(int pin)
{
    return (pin == -1) || GPIO_IS_VALID_GPIO(pin);
}

static bool gpio_ok_out(int pin)
{
    return (pin == -1) || GPIO_IS_VALID_OUTPUT_GPIO(pin);
}

static esp_err_t validate_camera_pins(const camera_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    // Pins that must be output-capable.
    if (!gpio_ok_out(cfg->pin_xclk) || !gpio_ok_out(cfg->pin_sccb_sda) || !gpio_ok_out(cfg->pin_sccb_scl) ||
        !gpio_ok_out(cfg->pin_pwdn) || !gpio_ok_out(cfg->pin_reset)) {
        ESP_LOGE(TAG,
                 "invalid output GPIO(s): xclk=%d sda=%d scl=%d pwdn=%d reset=%d",
                 cfg->pin_xclk, cfg->pin_sccb_sda, cfg->pin_sccb_scl, cfg->pin_pwdn, cfg->pin_reset);
        return ESP_ERR_INVALID_ARG;
    }

    // DVP pins are inputs.
    if (!gpio_ok_in(cfg->pin_d0) || !gpio_ok_in(cfg->pin_d1) || !gpio_ok_in(cfg->pin_d2) || !gpio_ok_in(cfg->pin_d3) ||
        !gpio_ok_in(cfg->pin_d4) || !gpio_ok_in(cfg->pin_d5) || !gpio_ok_in(cfg->pin_d6) || !gpio_ok_in(cfg->pin_d7) ||
        !gpio_ok_in(cfg->pin_pclk) || !gpio_ok_in(cfg->pin_vsync) || !gpio_ok_in(cfg->pin_href)) {
        ESP_LOGE(TAG,
                 "invalid input GPIO(s): d0=%d d1=%d d2=%d d3=%d d4=%d d5=%d d6=%d d7=%d pclk=%d vsync=%d href=%d",
                 cfg->pin_d0, cfg->pin_d1, cfg->pin_d2, cfg->pin_d3, cfg->pin_d4, cfg->pin_d5, cfg->pin_d6, cfg->pin_d7,
                 cfg->pin_pclk, cfg->pin_vsync, cfg->pin_href);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static void cfg_set_defaults(cam_uploader_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->interval_sec = 60;
    cfg->url[0] = '\0';
    cfg->voltage_url[0] = '\0';
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
    } else {
        cfg->url[sizeof(cfg->url) - 1] = '\0';
        normalize_url_inplace(cfg->url, sizeof(cfg->url));
    }

    size_t vurl_len = sizeof(cfg->voltage_url);
    err = nvs_get_str(h, NVS_KEY_VOLTAGE_URL, cfg->voltage_url, &vurl_len);
    if (err != ESP_OK) {
        cfg->voltage_url[0] = '\0';
    } else {
        cfg->voltage_url[sizeof(cfg->voltage_url) - 1] = '\0';
        normalize_url_inplace(cfg->voltage_url, sizeof(cfg->voltage_url));
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
        err = nvs_set_str(h, NVS_KEY_VOLTAGE_URL, cfg->voltage_url);
    }
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
    normalize_url_inplace(cleaned.url, sizeof(cleaned.url));
    cleaned.voltage_url[sizeof(cleaned.voltage_url) - 1] = '\0';
    normalize_url_inplace(cleaned.voltage_url, sizeof(cleaned.voltage_url));
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

    esp_err_t last_err = ESP_FAIL;
    for (size_t i = 0; i < (sizeof(s_cam_model_try_list) / sizeof(s_cam_model_try_list[0])); i++) {
        const cam_model_pins_t *m = &s_cam_model_try_list[i];

        camera_config_t config = {
            .ledc_channel = LEDC_CHANNEL_0,
            .ledc_timer = LEDC_TIMER_0,
            .pin_d0 = m->pin_d0,
            .pin_d1 = m->pin_d1,
            .pin_d2 = m->pin_d2,
            .pin_d3 = m->pin_d3,
            .pin_d4 = m->pin_d4,
            .pin_d5 = m->pin_d5,
            .pin_d6 = m->pin_d6,
            .pin_d7 = m->pin_d7,
            .pin_xclk = m->pin_xclk,
            .pin_pclk = m->pin_pclk,
            .pin_vsync = m->pin_vsync,
            .pin_href = m->pin_href,
            .pin_sccb_sda = m->pin_sccb_sda,
            .pin_sccb_scl = m->pin_sccb_scl,
            .pin_pwdn = m->pin_pwdn,
            .pin_reset = m->pin_reset,
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

        ESP_LOGI(TAG,
                 "trying camera model %s; pins: d0=%d d1=%d d2=%d d3=%d d4=%d d5=%d d6=%d d7=%d xclk=%d pclk=%d vsync=%d href=%d sda=%d scl=%d pwdn=%d reset=%d",
                 m->name,
                 config.pin_d0, config.pin_d1, config.pin_d2, config.pin_d3, config.pin_d4, config.pin_d5, config.pin_d6,
                 config.pin_d7, config.pin_xclk, config.pin_pclk, config.pin_vsync, config.pin_href, config.pin_sccb_sda,
                 config.pin_sccb_scl, config.pin_pwdn, config.pin_reset);

        esp_err_t pin_err = validate_camera_pins(&config);
        if (pin_err != ESP_OK) {
            ESP_LOGW(TAG, "skip model %s due to invalid GPIO", m->name);
            last_err = pin_err;
            continue;
        }

        esp_err_t err = esp_camera_init(&config);
        if (err == ESP_OK) {
            sensor_t *s = esp_camera_sensor_get();
            if (s) {
                s->set_framesize(s, FRAMESIZE_QVGA);
            }

            s_camera_inited = true;
            ESP_LOGI(TAG, "camera initialized with model %s", m->name);
            return ESP_OK;
        }

        last_err = err;
        ESP_LOGW(TAG, "model %s failed: 0x%x (%s)", m->name, err, esp_err_to_name(err));

        // Best-effort cleanup in case the driver partially initialized.
        (void)esp_camera_deinit();
    }

    ESP_LOGE(TAG, "all camera models failed; last error: 0x%x (%s)", last_err, esp_err_to_name(last_err));
    return last_err;
}

static esp_err_t http_post_jpeg(const char *url, const uint8_t *buf, size_t len)
{
    if (!url || url[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    // Guard against percent-encoded URL slipping through.
    char url_buf[256];
    const char *use_url = url;
    if (strchr(url, '%') && url_percent_decode(url, url_buf, sizeof(url_buf))) {
        if (strncmp(url_buf, "http://", 7) == 0 || strncmp(url_buf, "https://", 8) == 0) {
            use_url = url_buf;
        }
    }

    esp_http_client_config_t config = {
        .url = use_url,
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

static esp_err_t read_supply_voltage_mv(int *out_mv)
{
    if (!out_mv) {
        return ESP_ERR_INVALID_ARG;
    }

    adc_oneshot_unit_handle_t unit = NULL;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = VBAT_ADC_UNIT,
    };
    esp_err_t err = adc_oneshot_new_unit(&init_cfg, &unit);
    if (err != ESP_OK) {
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = VBAT_ADC_ATTEN,
    };
    err = adc_oneshot_config_channel(unit, VBAT_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        adc_oneshot_del_unit(unit);
        return err;
    }

    int raw = 0;
    err = adc_oneshot_read(unit, VBAT_ADC_CHANNEL, &raw);

    int mv = 0;
    adc_cali_handle_t cali = NULL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = VBAT_ADC_UNIT,
        .chan = VBAT_ADC_CHANNEL,
        .atten = VBAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali) == ESP_OK) {
        calibrated = true;
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = VBAT_ADC_UNIT,
        .chan = VBAT_ADC_CHANNEL,
        .atten = VBAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &cali) == ESP_OK) {
        calibrated = true;
    }
#endif

    if (err == ESP_OK) {
        if (calibrated && cali) {
            if (adc_cali_raw_to_voltage(cali, raw, &mv) != ESP_OK) {
                mv = (raw * VBAT_APPROX_FULLSCALE_MV) / 4095;
            }
        } else {
            mv = (raw * VBAT_APPROX_FULLSCALE_MV) / 4095;
        }

        mv = (mv * VBAT_DIVIDER_NUM) / VBAT_DIVIDER_DEN;
        *out_mv = mv;
    }

    if (calibrated && cali) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        (void)adc_cali_delete_scheme_curve_fitting(cali);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        (void)adc_cali_delete_scheme_line_fitting(cali);
#endif
    }
    adc_oneshot_del_unit(unit);
    return err;
}

static esp_err_t http_post_voltage_mv(const char *url, int voltage_mv)
{
    if (!url || url[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    char url_buf[256];
    const char *use_url = url;
    if (strchr(url, '%') && url_percent_decode(url, url_buf, sizeof(url_buf))) {
        if (strncmp(url_buf, "http://", 7) == 0 || strncmp(url_buf, "https://", 8) == 0) {
            use_url = url_buf;
        }
    }

    esp_http_client_config_t config = {
        .url = use_url,
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

    char body[32];
    snprintf(body, sizeof(body), "%d", voltage_mv);

    esp_http_client_set_header(client, "Content-Type", "text/plain");
    esp_http_client_set_post_field(client, body, (int)strlen(body));

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Voltage POST failed: %s", esp_err_to_name(err));
        return err;
    }
    if (status < 200 || status >= 300) {
        ESP_LOGW(TAG, "Voltage POST http status=%d", status);
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
            if (cfg.voltage_url[0] != '\0') {
                int voltage_mv = 0;
                esp_err_t v_err = read_supply_voltage_mv(&voltage_mv);
                if (v_err == ESP_OK) {
                    (void)http_post_voltage_mv(cfg.voltage_url, voltage_mv);
                } else {
                    ESP_LOGW(TAG, "read voltage failed: %s", esp_err_to_name(v_err));
                }
            }

            size_t frame_len = fb->len;
            esp_err_t post_err = http_post_jpeg(cfg.url, fb->buf, fb->len);
            esp_camera_fb_return(fb);

            int64_t dt_ms = (esp_timer_get_time() - t0) / 1000;
            if (post_err == ESP_OK) {
                ESP_LOGI(TAG, "uploaded %u bytes in %lld ms", (unsigned)frame_len, (long long)dt_ms);
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
