#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_reg.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_https_ota.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "wifi_helper.h"
#include "mqtt_helper.h"
#include "crc8.h"

static const char *TAG = "app";

static const char* ota_url = "http://raspberrypi.fritz.box:8032/esp32/ion1.bin";

#define CALIBRATION_FILE "/spiffs/calibration.bin"

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

#define RX_BUF_SIZE (1024)

static const int TURN_ON_BIT = BIT0;
static const int TURN_OFF_BIT = BIT1;
static const int CALIBRATE_BIT = BIT2;

static EventGroupHandle_t controlEventGroup;


struct messageType {
    // Payload length is indicated by one nibble, so max value 0xF (15).
    // Payload length excludes the starting byte, 2 header bytes, command byte, and crc byte.
    // So total length = payload + 5, and max length is 15 + 5 = 20.
    uint8_t data[20];
    uint8_t length;

    // Header values, we use -1 as 'unset'.
    uint8_t target;
    uint8_t source;
    int8_t type;
    int8_t size;
};

static void ota_task(void * pvParameter) {
    ESP_LOGI(TAG, "Starting OTA update...");

    esp_http_client_config_t config = {};
    config.url = ota_url;

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

messageType readMessage() {

    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE);

    bool escaping = false;    

    messageType message = {};
    message.target = -1;
    message.source = -1;
    message.type = -1;
    message.size = -1;

    while(true) {
        size_t rxReady = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, &rxReady));
        if(rxReady == 0) {
            // Always wait for at least one byte, even if there is none available.
            rxReady = 1;
        }
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, rxReady, 1000 / portTICK_RATE_MS);

        for(int bufferPos = 0 ; bufferPos < rxBytes; bufferPos++) {

            uint8_t byteRead = data[bufferPos];
            ESP_LOGD(TAG, "Read byte: %d", byteRead);

            uint8_t input[2] = {};
            uint8_t inputLen = 0;

            if(escaping) {
                input[0] = 0x10;
                inputLen += 1;
                if(byteRead != 0x10) {
                    input[1] = byteRead;
                    inputLen += 1;
                    if(message.length != 0) {
                        // Unescaped 0x10, reset
                        if(inputLen > 0) {
                            ESP_LOGI(TAG, "Incomplete:");
                            ESP_LOG_BUFFER_HEX(TAG, message.data, message.length);
                        }
                        message.length = 0;
                        message.target = -1;
                        message.source = -1;
                        message.type = -1;
                        message.size = -1;
                    }
                }
                escaping = false;
            } else if(byteRead == 0x10) {
                escaping = true;
                // Message start byte, we need to check the next input byte to decide what to do.
                continue;
            } else {
                // Not a message start byte, and we're not escaping, so just parse it normally.
                input[0] = byteRead;
                inputLen += 1;          
            }

            for(int pos = 0; pos < inputLen; pos++) {
                uint8_t value = input[pos];
                uint8_t low = value & 0x0f;
                uint8_t high = value >> 4;

                if(message.length == 0) {
                    // First byte in a new message
                    if(value == 0x00) {
                        // Ignore single '00' with no leading '10', which is sent by display to wake up system.
                        continue;
                    }
                } else if (message.length == 1) {
                    // First nibble is always message target.
                    message.target = high;
                    // Second nibble is always message type.
                    message.type = low;
                } else if (message.length == 2) {
                    if( message.type == 0x00) {
                        message.size = 3;
                    } else {
                        message.source = high;
                        if(message.type == 0x03 || message.type == 0x04) {
                            message.size = 4;
                        } else {
                            message.size = low + 5;
                        }
                    }
                }
                message.data[message.length++] = value;

                if(message.length > 2 && message.length == message.size) {
                    free(data);

                    // ESP_LOGI(TAG, "Message received:");
                    // ESP_LOG_BUFFER_HEX(TAG, message.data, message.length);
                    // ESP_LOGI(TAG, "Tgt:%d, Src:%d, Type:%d, Size:%d", message.target, message.source, message.type, message.size);

                    // uint8_t crc = crc8_bow( message.data, message.length - 1);

                    // ESP_LOGI(TAG, "CRC(calc): %02x", crc);

                    return message;
                }
            }
        }
    }
}

void writeMessage(uint8_t* message, uint8_t messageLen) {

    // First create the full message, unescaped, includig crc.
    uint8_t data[20];
    data[0] = 0x10;
    memcpy(data + 1, message, messageLen);
    data[messageLen + 1] = crc8_bow(data, messageLen + 1);

    // Now create an escaped copy
    uint8_t escaped[20*2];
    uint8_t outPos = 0;
    escaped[outPos++] = 0x10;
    for(uint8_t inPos = 1; inPos < messageLen + 2; inPos++) {
        escaped[outPos++] = data[inPos];
        if(data[inPos] == 0x10) {
            escaped[outPos++] = 0x10;
        }
    }
    uart_write_bytes(UART_NUM_2, escaped, outPos);
}

void exchange(uint8_t* cmd, size_t cmdLen) {
    writeMessage(cmd, cmdLen);

    messageType message;
    do {
        message = readMessage();
    } while (message.target != 0x02 && message.target != 0x0C);
    
    if(message.data[3] != cmd[2]) { // Watch out, cmd doesn't include the leading 0x10
        ESP_LOGE(TAG, "Wrong reply, expected %02x, got %02x",cmd[2], message.data[3]);
    } else {
        // ESP_LOGI(TAG, "<OK!");
    }
}

bool handleMotorMessage() {
    messageType message;
    do {
        message = readMessage();
    } while (message.target != 0x02 && message.target != 0x0C);

    if(message.data[0] == 0x10 && message.data[1] == 0x20) { // Handoff back to battery
        // ESP_LOGI(TAG, "|HNDF");
        return true; // Control back to us
    } else if(message.data[0] == 0x10 && message.data[1] == 0x21 && message.data[2] == 0x01 && message.data[3] == 0x12) { // MYSTERY BATTERY COMMAND 12
        // ESP_LOGI(TAG, "|BT:12");
        uint8_t cmd[] = {0x02, 0x20, 0x12};
        writeMessage(cmd, sizeof(cmd));
        return false;
    } else if(message.data[0] == 0x10 && message.data[1] == 0x21 && message.data[2] == 0x00 && message.data[3] == 0x11) { // MYSTERY BATTERY COMMAND 11
        // ESP_LOGI(TAG, "|BT:11");
        uint8_t cmd[] = {0x02, 0x20, 0x11};
        writeMessage(cmd, sizeof(cmd));
        return false;
    } else if(message.data[0] == 0x10 && message.data[1] == 0x21 && message.data[2] == 0x04 && message.data[3] == 0x08 && message.data[5] == 0x38 && message.data[7] == 0x3a) { // GET DATA 9438283a
        // ESP_LOGI(TAG, "|GET-38-3a");
        uint8_t cmd[] = {0x02, 0x2b, 0x08, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // Data (last 10 bytes) to be replaced

        struct stat st;
        if(stat(CALIBRATION_FILE, &st) == 0) {
            FILE* fp = fopen(CALIBRATION_FILE, "r");
            if (fp == NULL) {
                ESP_LOGE(TAG, "Failed to open calibration file for reading");
                return false;
            }
            fread(cmd + 4, 1, 10, fp);
            fclose(fp);
        } else {
            // Backup data
            uint8_t data[] = {0x94, 0x38, 0x4b, 0x15, 0x28, 0x3a, 0x3e, 0x91, 0x79, 0x50}; // This needs to be good calibration data!
            memcpy(cmd + 4, data, 10);
        }

        writeMessage(cmd, sizeof(cmd));
        return false;
    } else if(message.data[0] == 0x10 && message.data[1] == 0x21 && message.data[2] == 0x0a && message.data[3] == 0x09 && message.data[5] == 0xc0 && message.data[9] == 0xc1) {  // PUT DATA c0/c1
        // ESP_LOGI(TAG, "|PUT-c0-c1");
        uint8_t cmd[] = {0x02, 0x21, 0x09, 0x00};
        writeMessage(cmd, sizeof(cmd));
        return false;
    } else if(message.data[0] == 0x10 && message.data[1] == 0x21 && message.data[2] == 0x0a && message.data[3] == 0x09 && message.data[5] == 0x38 && message.data[9] == 0x3a) { // PUT DATA 38/3a
        // ESP_LOGI(TAG, "|PUT-38-3a");

        FILE* fp = fopen(CALIBRATION_FILE, "w");
        if (fp == NULL) {
            ESP_LOGE(TAG, "Failed to open calibration file for writing");
            return false;
        }
        fwrite(message.data + 4, 1, 10, fp);
        fclose(fp);

        uint8_t cmd[] = {0x02, 0x21, 0x09, 0x00};
        writeMessage(cmd, sizeof(cmd));
        return false;
    } else if(message.data[0] == 0x10 && message.data[1] == 0xc1 && message.data[2] == 0x00 && message.data[3] == 0x20) { // GET DISPLAY SERIAL
        // ESP_LOGI(TAG, "|SER");
        uint8_t cmd[] = {0x02, 0xc8, 0x20, 0x15, 0x27, 0x10, 0x00, 0x00, 0x00, 0x06, 0x66};
        writeMessage(cmd, sizeof(cmd));
        return false;
    }

    ESP_LOGI(TAG, "Unexpected:");
    ESP_LOG_BUFFER_HEX(TAG, message.data, message.length);

    return false;
}

void handoff() {

    uint8_t cmd[] = {0x00}; // HANDOFF to motor
    writeMessage(cmd, sizeof(cmd));

    while(!handleMotorMessage()) {
    }
    // print('Control returned to us')
}

void init_spiffs() {
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t spiffs_conf = {};
    spiffs_conf.base_path = "/spiffs";
    spiffs_conf.partition_label = NULL;
    spiffs_conf.max_files = 5;
    spiffs_conf.format_if_mount_failed = true;

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_conf));

    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info(spiffs_conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
}

void init_uart() {
    uart_config_t uart_config = {};
    uart_config.baud_rate = 9600;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;

    uart_intr_config_t uart_intr = {};
    uart_intr.intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M
                               | UART_RXFIFO_TOUT_INT_ENA_M
                               | UART_RXFIFO_OVF_INT_ENA_M
                               | UART_BRK_DET_INT_ENA_M
                               | UART_PARITY_ERR_INT_ENA_M;

    uart_intr.rxfifo_full_thresh = 1; // This should speed things up.
    uart_intr.rx_timeout_thresh = 10;
    uart_intr.txfifo_empty_intr_thresh = 10;

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_intr_config(UART_NUM_2, &uart_intr));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void my_task(void *pvParameter) {

    init_spiffs();

    init_uart();

    while(true) {

        EventBits_t bits = xEventGroupWaitBits(controlEventGroup, TURN_ON_BIT | CALIBRATE_BIT, true, false, portMAX_DELAY);

        if((bits & CALIBRATE_BIT) != 0) {
            // ESP_LOGI(TAG, ">MTR:CAL");
            uint8_t cmdc1[] = {0x01, 0x20, 0x35};
            exchange(cmdc1, sizeof(cmdc1));

            // ESP_LOGI(TAG, ">HNDF");
            handoff();

            // ESP_LOGI(TAG, ">MTR:GET/CAL?");
            uint8_t cmdc2[] = {0x01, 0x22, 0x08, 0x00, 0xdf};
            exchange(cmdc2, sizeof(cmdc2));

            handoff();

            continue;
        }

        // ESP_LOGI(TAG, ">MTR:ON");
        uint8_t cmd1[] = {0x01, 0x20, 0x30};
        exchange(cmd1, sizeof(cmd1));

        // ESP_LOGI(TAG, ">HNDF");
        handoff();

        // ESP_LOGI(TAG, ">PUT");
        uint8_t cmd2[] = {0x01, 0x28, 0x09, 0x94, 0xb0, 0x09, 0xc4, 0x14, 0xb1, 0x01, 0x14}; // PUT DATA (2500|27.6)
        exchange(cmd2, sizeof(cmd2));

        // ESP_LOGI(TAG, ">HNDF");
        handoff();

        // ESP_LOGI(TAG, ">ASS:ON");
        uint8_t cmd3[] = {0x01, 0x20, 0x32};
        exchange(cmd3, sizeof(cmd3));

        // ESP_LOGI(TAG, ">HNDF");
        handoff();

        // ESP_LOGI(TAG, ">ASS:POW");
        uint8_t cmd4[] = {0x01, 0x21, 0x34, 0x03}; // SET ASSIST LEVEL 03 > POWER!
        exchange(cmd4, sizeof(cmd4));

        while(true) {
            EventBits_t bits = xEventGroupWaitBits(controlEventGroup, TURN_OFF_BIT, true, true, 0);

            if((bits & TURN_OFF_BIT) != 0) {
                break;
            }
    
            // ESP_LOGI(TAG, ">HNDF");
            handoff();
        }

        // ESP_LOGI(TAG, ">ASS:OFF");
        uint8_t cmd5[] = {0x01, 0x20, 0x33};
        exchange(cmd5, sizeof(cmd5));

        // ESP_LOGI(TAG, ">HNDF");
        handoff();

        // Maybe wait for cmd 12?

        // ESP_LOGI(TAG, ">MTR:OFF");
        uint8_t cmd6[] = {0x01, 0x21, 0x31, 0x00};
        exchange(cmd6, sizeof(cmd6));

        // ESP_LOGI(TAG, ">HNDF");
        handoff();

        // Maybe wait for cmd 11?

        while(true) {
            EventBits_t bits = xEventGroupWaitBits(controlEventGroup, TURN_ON_BIT, false, true, 0);

            if((bits & TURN_ON_BIT) != 0) {
                break;
            }

            // ESP_LOGI(TAG, ">HNDF");
            handoff();
        }
    }

    vTaskDelete(NULL);
}

static void subscribeTopics() {
    subscribeDevTopic("$update");
    subscribeDevTopic("$command");
}

static void handleMessage(const char* topic1, const char* topic2, const char* topic3, const char* data) {
    if(
        strcmp(topic1, "$update") == 0 && 
        topic2 == NULL && 
        topic3 == NULL
    ) {
        xEventGroupSetBits(controlEventGroup, TURN_OFF_BIT);
        xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
    }

    if(
        strcmp(topic1, "$command") == 0 && 
        topic2 == NULL && 
        topic3 == NULL
    ) {
        if(strcmp(data, "on") == 0) {
            xEventGroupSetBits(controlEventGroup, TURN_ON_BIT);
        } else if(strcmp(data, "off") == 0) {
            xEventGroupSetBits(controlEventGroup, TURN_OFF_BIT);
        } else if(strcmp(data, "cal") == 0) {
            xEventGroupSetBits(controlEventGroup, CALIBRATE_BIT);
        }
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

    controlEventGroup = xEventGroupCreate();

    wifiStart();
    wifiWait();

    mqttStart(subscribeTopics, handleMessage);    
    mqttWait();

    ESP_LOGI(TAG, "MQTT started");

    xTaskCreatePinnedToCore(my_task, "my_task", 4096, NULL, 5, NULL, APP_CPU_NUM);
}
