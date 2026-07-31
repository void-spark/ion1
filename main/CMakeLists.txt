idf_component_register(SRC_DIRS "." "states"
    INCLUDE_DIRS "."
    REQUIRES esp32_button driver esp_timer nvs_flash esp_adc
)