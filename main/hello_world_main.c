
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "lwip/inet.h"
#include "esp_mac.h"

#define EXAMPLE_ESP_WIFI_SSID      "ESP32_PROV"
#define EXAMPLE_ESP_WIFI_PASS      "12345678"
#define EXAMPLE_MAX_STA_CONN       4

#ifndef MIN
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif

static const char *TAG = "wifi_prov";
static esp_netif_t *ap_netif = NULL;
static esp_netif_t *sta_netif = NULL;
static httpd_handle_t server = NULL;

// HTML code for the configuration page
static const char *config_page_html = 
"<!DOCTYPE html>"
"<html>"
"<head>"
"<title>WiFi Configuration</title>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<style>"
"body {font-family: Arial, Helvetica, sans-serif;}"
"input[type=text], input[type=password] {"
"  width: 100%;"
"  padding: 12px 20px;"
"  margin: 8px 0;"
"  display: inline-block;"
"  border: 1px solid #ccc;"
"  box-sizing: border-box;"
"}"
"button {"
"  background-color: #4CAF50;"
"  color: white;"
"  padding: 14px 20px;"
"  margin: 8px 0;"
"  border: none;"
"  cursor: pointer;"
"  width: 100%;"
"}"
"button:hover {"
"  opacity: 0.8;"
"}"
".container {"
"  padding: 16px;"
"}"
"</style>"
"</head>"
"<body>"
"<h2>WiFi Configuration</h2>"
"<form action='/save' method='post'>"
"  <div class='container'>"
"    <label for='ssid'><b>WiFi SSID</b></label>"
"    <input type='text' placeholder='Enter SSID' name='ssid' required>"
"    <label for='password'><b>Password</b></label>"
"    <input type='password' placeholder='Enter Password' name='password' required>"
"    <button type='submit'>Connect</button>"
"  </div>"
"</form>"
"</body>"
"</html>";

// Event handler for WiFi events
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected from WiFi, trying to reconnect...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

// HTTP GET handler for root page
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_send(req, config_page_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// HTTP POST handler for saving WiFi credentials
static esp_err_t save_post_handler(httpd_req_t *req)
{
    char content[200];
    size_t recv_size = MIN(req->content_len, sizeof(content));
    
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    
    content[recv_size] = '\0';
    
    char ssid[32] = {0};
    char password[64] = {0};
    
    // Parse SSID
    char *ssid_ptr = strstr(content, "ssid=");
    if (ssid_ptr) {
        ssid_ptr += 5; // Skip "ssid="
        char *ssid_end = strchr(ssid_ptr, '&');
        if (ssid_end) {
            int len = MIN(ssid_end - ssid_ptr, sizeof(ssid) - 1);
            strncpy(ssid, ssid_ptr, len);
            ssid[len] = '\0';
        } else {
            strncpy(ssid, ssid_ptr, sizeof(ssid) - 1);
        }
    }
    
    // Parse password
    char *pass_ptr = strstr(content, "password=");
    if (pass_ptr) {
        pass_ptr += 9; // Skip "password="
        char *pass_end = strchr(pass_ptr, '&');
        if (!pass_end) {
            pass_end = pass_ptr + strlen(pass_ptr);
        }
        int len = MIN(pass_end - pass_ptr, sizeof(password) - 1);
        strncpy(password, pass_ptr, len);
        password[len] = '\0';
    }
    
    ESP_LOGI(TAG, "Received SSID: %s, Password: %s", ssid, password);
    
    // Save credentials to NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_creds", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_set_str(nvs_handle, "ssid", ssid);
        nvs_set_str(nvs_handle, "password", password);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Credentials saved to NVS");
    } else {
        ESP_LOGE(TAG, "Failed to open NVS");
    }
    
    // Connect to the new WiFi
    wifi_config_t wifi_config = {0};
    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, password);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());
    
    const char *response = "<html><body><h1>Connecting to WiFi...</h1><p>Device will now connect to the specified WiFi network.</p></body></html>";
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}

// Start web server
static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        // URI handler for root page
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root_uri);
        
        // URI handler for saving credentials
        httpd_uri_t save_uri = {
            .uri       = "/save",
            .method    = HTTP_POST,
            .handler   = save_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &save_uri);
        
        return server;
    }
    
    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

// Initialize SoftAP
static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    ap_netif = esp_netif_create_default_wifi_ap();
    sta_netif = esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(ap_netif, &ip_info);
    
    char ip_addr[16];
    inet_ntoa_r(ip_info.ip.addr, ip_addr, sizeof(ip_addr));
    ESP_LOGI(TAG, "SoftAP started with IP: %s", ip_addr);
    ESP_LOGI(TAG, "WiFi AP SSID: %s password: %s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

// Try to connect to previously saved WiFi
static bool try_saved_wifi(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_creds", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "NVS not initialized or no saved credentials");
        return false;
    }
    
    char ssid[32] = {0};
    char password[64] = {0};
    size_t ssid_len = sizeof(ssid);
    size_t pass_len = sizeof(password);
    
    err = nvs_get_str(nvs_handle, "ssid", ssid, &ssid_len);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No saved SSID found");
        nvs_close(nvs_handle);
        return false;
    }
    
    err = nvs_get_str(nvs_handle, "password", password, &pass_len);
    nvs_close(nvs_handle);
    
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No saved password found");
        return false;
    }
    
    ESP_LOGI(TAG, "Found saved credentials. Connecting to SSID: %s", ssid);
    
    wifi_config_t wifi_config = {0};
    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, password);
    
    // Ensure WiFi is in a configurable state
    esp_wifi_disconnect();
    esp_wifi_stop();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    
    return true;
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "Starting WiFi provisioning example");
    
    // Check if we have saved WiFi credentials
    nvs_handle_t nvs_handle;
    bool has_saved_creds = false;
    
    if (nvs_open("wifi_creds", NVS_READONLY, &nvs_handle) == ESP_OK) {
        char ssid[32] = {0};
        size_t ssid_len = sizeof(ssid);
        if (nvs_get_str(nvs_handle, "ssid", ssid, &ssid_len) == ESP_OK) {
            has_saved_creds = true;
        }
        nvs_close(nvs_handle);
    }
    
    if (has_saved_creds) {
        // If we have saved credentials, only initialize STA mode
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        
        sta_netif = esp_netif_create_default_wifi_sta();
        
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
        
        // Try to connect with saved credentials
        if (try_saved_wifi()) {
            ESP_LOGI(TAG, "Connected to saved WiFi network");
        } else {
            ESP_LOGI(TAG, "Failed to connect with saved credentials");
            // Fall back to provisioning mode
            ESP_LOGI(TAG, "No saved credentials found, starting provisioning mode");
            // Start web server for provisioning
            server = start_webserver();
            if (server) {
                ESP_LOGI(TAG, "Web server started. Connect to SSID '%s' with password '%s'", 
                        EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
                ESP_LOGI(TAG, "Then open http://192.168.4.1 in your browser to configure WiFi");
            }
        }
    } else {
        // No saved credentials, initialize in APSTA mode for provisioning
        wifi_init_softap();

        ESP_LOGI(TAG, "No saved credentials found, starting provisioning mode");
        // Start web server for provisioning
        server = start_webserver();
        if (server) {
            ESP_LOGI(TAG, "Web server started. Connect to SSID '%s' with password '%s'", 
                     EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
            ESP_LOGI(TAG, "Then open http://192.168.4.1 in your browser to configure WiFi");
        }
    }
    
    // Continue with the original example code
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // Commenting out the restart loop to keep the WiFi and web server running
    /*
    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
    */
}