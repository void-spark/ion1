#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_https_ota.h"
#include "nvs_flash.h"
#include "lwip/apps/sntp.h"
#include "wifi_helper.h"
#include "mqtt_helper.h"

static const char *TAG = "app";

static const char* ota_url = "http://raspberrypi.fritz.box:8032/esp32/ion1.bin";

static void ota_task(void * pvParameter) {
    ESP_LOGI(TAG, "Starting OTA update...");

    esp_http_client_config_t config = {
        .url = ota_url,
    };
    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware Upgrades Failed");
    }
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void my_task(void *pvParameter) {


    vTaskDelete(NULL);
}


static void subscribeTopics() {
    subscribeDevTopic("$update");
}

static void handleMessage(const char* topic1, const char* topic2, const char* topic3, const char* data) {

    if(
        strcmp(topic1, "$update") == 0 && 
        topic2 == NULL && 
        topic3 == NULL
    ) {
        xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
    }
}

extern "C" void app_main() {

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xTaskCreatePinnedToCore(my_task, "my_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL, APP_CPU_NUM);

    wifiStart();

    wifiWait();

    mqttStart(subscribeTopics, handleMessage);
    mqttWait();

    ESP_LOGI(TAG, "MQTT started");
}
