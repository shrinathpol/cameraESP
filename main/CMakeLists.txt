idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp32-camera esp_http_server esp_wifi esp_event nvs_flash esp_timer esp_psram heap app_update
)
