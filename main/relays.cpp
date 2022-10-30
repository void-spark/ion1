#include "driver/gpio.h"
#include "relays.h"

#if CONFIG_ION_LIGHT
    #define LIGHT_PIN ((gpio_num_t)CONFIG_ION_LIGHT_PIN)
    #if CONFIG_ION_LIGHT_PIN_INVERTED
        #define LIGHT_ON 0
        #define LIGHT_OFF 1
    #else
        #define LIGHT_ON 1
        #define LIGHT_OFF 0
    #endif
#endif

#if CONFIG_ION_RELAY
    #define RELAY_PIN ((gpio_num_t)CONFIG_ION_RELAY_PIN)
    #if CONFIG_ION_RELAY_PIN_INVERTED
        #define RELAY_ON 0
        #define RELAY_OFF 1
    #else
        #define RELAY_ON 1
        #define RELAY_OFF 0
    #endif
#endif

// Light on/off
static bool lightOn = false;

void setLight(bool value) {
    lightOn = value;
#if CONFIG_ION_LIGHT
    gpio_set_level(LIGHT_PIN, value ? LIGHT_ON : LIGHT_OFF);
#endif
}

bool getLight() {
    return lightOn;
}

void toggleLight() {
    setLight(!lightOn);
}

void initLight() {
#if CONFIG_ION_LIGHT
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = BIT64(LIGHT_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
#endif

    setLight(false);
}

void setRelay(bool value) {
#if CONFIG_ION_RELAY
    gpio_set_level(RELAY_PIN, value ? RELAY_ON : RELAY_OFF);
#endif
}

void initRelay() {
#if CONFIG_ION_RELAY
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = BIT64(RELAY_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    setRelay(false);
#endif
}
