idf_component_register(SRC_DIRS "." "states"
                       PRIV_REQUIRES esp32-button nvs_flash esp_driver_uart esp_driver_gpio esp_adc esp_timer
                       INCLUDE_DIRS ".")
