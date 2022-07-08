#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "blink.h"

#define FIRST_CPU PRO_CPU_NUM

#define LED_BUILTIN ((gpio_num_t)CONFIG_ION_LED_PIN)

static QueueHandle_t blinkQueue;

struct blinkType {
        size_t blinks;
        uint32_t onTime;
        uint32_t offTime;
};

void queueBlink(size_t blinks, uint32_t onTime, uint32_t offTime) {
    blinkType blink = {};
    blink.blinks = blinks;
    blink.onTime = onTime;
    blink.offTime = offTime;

    xQueueSend(blinkQueue, &blink, 0);
}

static void blinkTask(void *pvParameter) {
    while(true) {
        blinkType blink = {};
        if (xQueueReceive(blinkQueue, &blink, portMAX_DELAY) == pdTRUE) {
            for(int loop = 0; loop < blink.blinks; loop++) {
                gpio_set_level(LED_BUILTIN, 1);
                vTaskDelay(blink.onTime / portTICK_PERIOD_MS);
                gpio_set_level(LED_BUILTIN, 0);
                vTaskDelay(blink.offTime / portTICK_PERIOD_MS);
            }
        }
    }

    vTaskDelete(NULL);
}

void initBlink() {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = BIT64(LED_BUILTIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    blinkQueue = xQueueCreate(3, sizeof(blinkType));

    xTaskCreatePinnedToCore(blinkTask, "blinkTask", 2048, NULL, 5, NULL, FIRST_CPU);
}
