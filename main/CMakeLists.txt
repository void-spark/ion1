idf_component_register(SRC_DIRS "." "states"
    INCLUDE_DIRS "."
    REQUIRES esp32-button driver esp_timer nvs_flash esp_adc
)