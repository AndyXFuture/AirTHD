set(SOURCES main.c lvgl_gui.c lvgl_gui.h image.h esp_ibeacon_api.c ws2812.c sensor.c wifi_station.c)
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES nvs_flash bt fatfs esp_http_client json
                    REQUIRES lvgl_esp32_drivers lvgl_touch lvgl_tft lvgl lv_examples)

target_compile_definitions(${COMPONENT_LIB} PRIVATE LV_CONF_INCLUDE_SIMPLE=1)
