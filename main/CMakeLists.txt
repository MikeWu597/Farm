idf_component_register(SRCS "hello_world_main.c" "cam_uploader.c"
                       PRIV_REQUIRES spi_flash nvs_flash esp_wifi esp_http_server esp_http_client esp_timer esp-tls esp_adc
                       INCLUDE_DIRS "" "../sdk")