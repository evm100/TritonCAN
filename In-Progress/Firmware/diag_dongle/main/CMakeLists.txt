idf_component_register(
    SRCS
        "main.c"
        "robstride.c"
        "state.c"
        "can_task.c"
        "diag_tests.c"
        "oled.c"
        "argb.c"
        "battery.c"
        "web_task.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_adc
        esp_event
        esp_http_server
        esp_netif
        esp_timer
        esp_wifi
        json
        lwip
        nvs_flash
        esp_driver_i2c
        esp_driver_gpio
        esp_driver_twai
)

# Embed the single-page web UI as binary blobs.
target_add_binary_data(${COMPONENT_LIB} "web/index.html"   TEXT)
