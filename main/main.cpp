#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "button.h"
#include "bow.h"
#if CONFIG_ION_CU2
    #include "cu2.h"
#endif

static const char *TAG = "app";


#define FIRST_CPU PRO_CPU_NUM

#if SOC_CPU_CORES_NUM > 1
    #define SECOND_CPU APP_CPU_NUM
#else
    #define SECOND_CPU PRO_CPU_NUM
#endif

#define CALIBRATION_FILE "/spiffs/calibration.bin"

#if CONFIG_ION_BUTTON
    #define BUTTON (GPIO_NUM_0)
    #define BUTTON_EXT (GPIO_NUM_4)
#endif

#define LED_BUILTIN ((gpio_num_t)CONFIG_ION_LED_PIN)

static const int BUTTON_MODE_SHORT_PRESS_BIT = BIT0;
static const int BUTTON_MODE_LONG_PRESS_BIT = BIT1;
static const int BUTTON_LIGHT_SHORT_PRESS_BIT = BIT2;
static const int BUTTON_LIGHT_LONG_PRESS_BIT = BIT3;
#if CONFIG_ION_CU2
static const int CHECK_BUTTON_BIT = BIT4;
static const int DISPLAY_UPDATE_BIT = BIT5;
#endif
static const int IGNORE_HELD_BIT = BIT6;


static EventGroupHandle_t controlEventGroup;

enum control_state { IDLE, START_CALIBRATE, TURN_MOTOR_ON, MOTOR_ON, SET_ASSIST_LEVEL, TURN_MOTOR_OFF, MOTOR_OFF };

enum handleMotorMessageResult {
    // We got a handoff back, so we get to send the next message
    CONTROL_TO_US,
    // We had to reply to a motor message, so motor is next to send a message.
    CONTROL_TO_MOTOR,
    // We did not get a timely reply to a handoff message.
    HANDOFF_TIMEOUT
};

// Current assist level
static uint8_t level = 0x00;

// Speed in km/h*10
static uint16_t speed;

// Trip in 10m increments
static uint32_t trip;

// Light on/off
static bool lightOn = false;

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

static uint16_t toUint16(uint8_t *buffer, size_t offset) { 
    return ((uint16_t)buffer[offset] << 8) | ((uint16_t)buffer[offset + 1] << 0); 
}

static uint32_t toUint32(uint8_t *buffer, size_t offset) {
    return ((uint32_t)buffer[offset] << 24) | ((uint32_t)buffer[offset + 1] << 16) | ((uint32_t)buffer[offset + 2] << 8) | ((uint32_t)buffer[offset + 3] << 0);
}

static handleMotorMessageResult handleMotorMessage() {
    messageType message = {};
    readResult result;
    do {
        // Replies to handoff should be a lot quicker then 250ms
        result = readMessage(&message, 250 / portTICK_PERIOD_MS);
        if(result == MSG_TIMEOUT) {
            // Most likely the motor turned off, after we told it to (XHP, Toprun doesn't seem to stop)
            return HANDOFF_TIMEOUT;
        }
    } while(result != MSG_OK || message.target != 0x2);

    if(message.type == 0x0) { // Handoff back to battery
        // ESP_LOGI(TAG, "|HNDF");
        return CONTROL_TO_US;
    } else if(message.type == 0x4 && message.source == 0x0) {
        // PING
        // ESP_LOGI(TAG, "|PING");
        uint8_t cmd[] = {0x03, 0x20};
        writeMessage(cmd, sizeof(cmd));
        return CONTROL_TO_MOTOR;
    } else if(message.type == 0x1 && message.source == 0x0 && message.payloadSize == 1 && message.command == 0x12) {
        // MYSTERY BATTERY COMMAND 12
        // ESP_LOGI(TAG, "|BT:12");
        uint8_t cmd[] = {0x02, 0x20, 0x12};
        writeMessage(cmd, sizeof(cmd));
        return CONTROL_TO_MOTOR;
    } else if(message.type == 0x1 && message.source == 0x0 && message.payloadSize == 0 && message.command== 0x11) {
        // MYSTERY BATTERY COMMAND 11
        // ESP_LOGI(TAG, "|BT:11");
        uint8_t cmd[] = {0x02, 0x20, 0x11};
        writeMessage(cmd, sizeof(cmd));
        return CONTROL_TO_MOTOR;
    } else if(message.type == 0x1 && message.source == 0x0 && message.payloadSize == 2 && message.command== 0x08 && message.payload[1] == 0x2a) {
        // GET DATA 002a
        // ESP_LOGI(TAG, "|GET-2a");
        uint8_t cmd[] = {0x02, 0x24, 0x08, 0x00, 0x00, 0x2a, 0x01};
        writeMessage(cmd, sizeof(cmd));
        return CONTROL_TO_MOTOR;
    } else if(message.type == 0x1 && message.source == 0x0 && message.payloadSize == 4 && message.command== 0x08 && message.payload[1] == 0x38 && message.payload[3] == 0x3a) {
        // GET DATA 9438283a
        // ESP_LOGI(TAG, "|GET-38-3a");
        uint8_t cmd[] = {0x02, 0x2b, 0x08, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // Data (last 10 bytes) to be replaced

        struct stat st;
        if(stat(CALIBRATION_FILE, &st) == 0) {
            FILE *fp = fopen(CALIBRATION_FILE, "r");
            if(fp == NULL) {
                ESP_LOGE(TAG, "Failed to open calibration file for reading");
                return CONTROL_TO_MOTOR;
            }
            fread(cmd + 4, 1, 10, fp);
            fclose(fp);
        } else {
            // Backup data
            // Gold small test: 94 38 4b 13 28 3a 3e 98 ed f3
            uint8_t data[] = {0x94, 0x38, 0x4b, 0x15, 0x28, 0x3a, 0x3e, 0x91, 0x79, 0x50}; // This needs to be good calibration data!
            memcpy(cmd + 4, data, 10);
        }

        writeMessage(cmd, sizeof(cmd));
        return CONTROL_TO_MOTOR;
    } else if(message.type == 0x1 && message.source == 0x0 && message.payloadSize == 10 && message.command == 0x09 && message.payload[1] == 0xc0 && message.payload[5] == 0xc1) {
        // PUT DATA c0/c1
        // ESP_LOGI(TAG, "|PUT-c0-c1");

        speed = toUint16(message.payload, 2);
        trip = toUint32(message.payload, 6);

        uint8_t cmd[] = {0x02, 0x21, 0x09, 0x00};
        writeMessage(cmd, sizeof(cmd));
#if CONFIG_ION_CU2
        // Notify display update
        xEventGroupSetBits(controlEventGroup, DISPLAY_UPDATE_BIT);
#endif
        return CONTROL_TO_MOTOR;
    } else if(message.type == 0x1 && message.source == 0x0 && message.payloadSize == 10 && message.command == 0x09 && message.payload[1] == 0x38 && message.payload[5] == 0x3a) {
        // PUT DATA 38/3a
        // ESP_LOGI(TAG, "|PUT-38-3a");

        FILE *fp = fopen(CALIBRATION_FILE, "w");
        if(fp == NULL) {
            ESP_LOGE(TAG, "Failed to open calibration file for writing");
            return CONTROL_TO_MOTOR;
        }
        fwrite(message.payload, 1, 10, fp);
        fclose(fp);

        uint8_t cmd[] = {0x02, 0x21, 0x09, 0x00};
        writeMessage(cmd, sizeof(cmd));
        return CONTROL_TO_MOTOR;
    }

    ESP_LOGI(TAG, "Unexpected: Tgt:%d, Src:%d, Type:%d, Command:%d", message.target, message.source, message.type, message.command);
    ESP_LOG_BUFFER_HEX(TAG, message.payload, message.payloadSize);

    return CONTROL_TO_MOTOR;
}

static bool handoff() {

    uint8_t cmd[] = {0x00}; // HANDOFF to motor
    writeMessage(cmd, sizeof(cmd));

    while(true) {
        handleMotorMessageResult result = handleMotorMessage();
        if(result == CONTROL_TO_US) {
            return true;
        }
        if(result == HANDOFF_TIMEOUT) {
            return false;
        }
    }
}

static void init_spiffs() {
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t spiffs_conf = {};
    spiffs_conf.base_path = "/spiffs";
    spiffs_conf.partition_label = NULL;
    spiffs_conf.max_files = 5;
    spiffs_conf.format_if_mount_failed = true;

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_conf));

    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info(spiffs_conf.partition_label, &total, &used);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    
    struct stat st;
    if(stat(CALIBRATION_FILE, &st) == 0) {
        FILE *fp = fopen(CALIBRATION_FILE, "r");
        if(fp == NULL) {
            ESP_LOGE(TAG, "Failed to open calibration file for reading");
        } else {
            uint8_t data[10];
            size_t read = fread(data, 1, sizeof(data), fp);
            fclose(fp);
            ESP_LOGI(TAG, "Calibration file found. Size: %lu, content:", st.st_size);        
            ESP_LOG_BUFFER_HEX(TAG, data, read);
        }
    }
}

// TODO
static void readTask(void *pvParameter) {
    // To RTOS stream
    while(true) {

    }

    vTaskDelete(NULL);
}

static void my_task(void *pvParameter) {

    init_spiffs();

    initUart();

#if CONFIG_ION_CU2
    initCu2(controlEventGroup, 
            BUTTON_MODE_SHORT_PRESS_BIT, 
            BUTTON_MODE_LONG_PRESS_BIT, 
            BUTTON_LIGHT_SHORT_PRESS_BIT, 
            BUTTON_LIGHT_LONG_PRESS_BIT, 
            CHECK_BUTTON_BIT,
            IGNORE_HELD_BIT
    );
#endif

    // The state we're in
    control_state state = IDLE;
    // The step of the state we're in, most states follow a sequence of commands
    uint8_t step = 0;
    bool motorHandoffs = false;
    bool assistOn = false;

    while(true) {

        // TODO:
        // Use 'step' for calibrate, double check logic of all states.
        // How do we go back to IDLE state? (timeout when level = 0?)

        // TODO:
        // Instead .. ? can we wait on event bits AND rx?
        // Serial should lead, buttons are uncommon.
        // Can we generate a eventgroup bit from uart?

        EventBits_t buttonBits = xEventGroupWaitBits(controlEventGroup, BUTTON_MODE_SHORT_PRESS_BIT | BUTTON_MODE_LONG_PRESS_BIT | BUTTON_LIGHT_SHORT_PRESS_BIT | BUTTON_LIGHT_LONG_PRESS_BIT, true, false, 0);
        const bool modeShortPress = (buttonBits & BUTTON_MODE_SHORT_PRESS_BIT) != 0;
        const bool modeLongPress = (buttonBits & BUTTON_MODE_LONG_PRESS_BIT) != 0;
        const bool lightShortPress = (buttonBits & BUTTON_LIGHT_SHORT_PRESS_BIT) != 0;
        const bool lightLongPress = (buttonBits & BUTTON_LIGHT_LONG_PRESS_BIT) != 0;

        if(lightShortPress) {
            lightOn = !lightOn;
#if CONFIG_ION_CU2
            xEventGroupSetBits(controlEventGroup, DISPLAY_UPDATE_BIT); 
#endif
        }

#if CONFIG_ION_CU2
        EventBits_t bits = xEventGroupWaitBits(controlEventGroup, CHECK_BUTTON_BIT | DISPLAY_UPDATE_BIT, false, false, 0);
        if((bits & CHECK_BUTTON_BIT) != 0) {
            xEventGroupClearBits(controlEventGroup, CHECK_BUTTON_BIT);
            buttonCheck();
        } else if((bits & DISPLAY_UPDATE_BIT) != 0) {
            xEventGroupClearBits(controlEventGroup, DISPLAY_UPDATE_BIT);
            showState(level, lightOn, speed, trip);
        } else 
#endif
        if(state == IDLE) {
            if(modeShortPress) {
                queueBlink(1, 800, 200);
                state = TURN_MOTOR_ON;
#if CONFIG_ION_CU2
                step = 0;
#else
                step = 5;
#endif
            } else {
                messageType message = {};
                readResult result = readMessage(&message, 50 / portTICK_PERIOD_MS );
                if(result == MSG_WAKEUP) {
                    ESP_LOGI(TAG, "Wakeup!");
                    xEventGroupSetBits(controlEventGroup, IGNORE_HELD_BIT); 
                    state = TURN_MOTOR_ON;
#if CONFIG_ION_CU2
                    step = 0;
#else
                    step = 5;
#endif
                } else if(result == MSG_OK){
                    ESP_LOGI(TAG, "Incoming: Tgt:%d, Src:%d, Type:%d, Command:%d", message.target, message.source, message.type, message.command);
                    ESP_LOG_BUFFER_HEX(TAG, message.payload, message.payloadSize);
                }
            }
        } else if(state == TURN_MOTOR_ON) {
#if CONFIG_ION_CU2
            if(step == 0) {
                // Button check command with a special value, maybe just resets
                // default/display? Or sets timeout? Or initializes display 'clock'?
                uint8_t cmd[] = {0xc1, 0x21, 0x22, 0x80};

                messageType message = {};
                readResult result = exchange(cmd, sizeof(cmd), &message, 225 / portTICK_PERIOD_MS );
                step++;
            } else if(step == 1) {
                // Update display
                displayUpdate(false, ASS_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_SOLID, BLNK_SOLID, true, 25, 0xccc, 0xccccc);
                step++;
            } else if(step == 2) {
                // Unknown command which is always the same and always sent to the
                // display at this point.
                uint8_t cmd[] = {0xc1, 0x22, 0x25, 0x04, 0x08};
                exchange(cmd, sizeof(cmd));
                step++;
            } else if(step == 3) {
                // First normal button check command, after this should run every 100ms.
                buttonCheck();
                startButtonCheck();
                step++;
            } else if(step == 4) {
                // Set default display, which is shown if the display isn't updated for a bit (?)
                displayUpdate(true, ASS_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, false, 10, 0xccc, 0xccccc);
                step++;
            } else 
#endif
            if(step == 5) {
                // Motor on
                uint8_t cmd[] = {0x01, 0x20, 0x30};

                messageType message = {};
                // Original BMS seems to repeat handoff till the motor responds, with 41ms between commands, but this should also work.
                exchange(cmd, sizeof(cmd), &message, 41 / portTICK_PERIOD_MS);
                motorHandoffs = true;
                step++;
            } else if(step == 6) {
                // Put data, which is common after motor on, left value is unknown,
                // right is voltage.
                uint8_t cmd[] = {0x01, 0x28, 0x09, 0x94, 0xb0, 0x09, 0xc4, 0x14, 0xb1, 0x01, 0x14}; // PUT DATA (2500|27.6)
                exchange(cmd, sizeof(cmd));
                state = MOTOR_ON;
                step = 0;
            }
        } else if(state == MOTOR_ON) {
            if(level == 0x00 && lightOn == false && lightLongPress) {
                queueBlink(10, 100, 100);
                state = START_CALIBRATE;
            } else if(modeShortPress) {
                if(level <= 0x02) {
                    queueBlink(level + 1, 250, 200);
                    level++;
                    state = SET_ASSIST_LEVEL;
                    step = 0;
                } else if(level == 0x03) {
                    queueBlink(4, 400, 200);
                    level = 0x00;
                    state = TURN_MOTOR_OFF;
                    step = 0;
                }
            }
        } else if(state == START_CALIBRATE) {
            // Start calibration
            uint8_t cmdc1[] = {0x01, 0x20, 0x35};
            exchange(cmdc1, sizeof(cmdc1));
            handoff();

            // Put data, which is common after calibrate. No idea what it's for.
            uint8_t cmdc2[] = {0x01, 0x22, 0x08, 0x00, 0xdf};
            exchange(cmdc2, sizeof(cmdc2));
            handoff();

            // BMS actually stops listening here, it ignores (some?) motor messages.
            state = MOTOR_ON;
        } else if(state == SET_ASSIST_LEVEL) {
            if(!assistOn) {
                // Assist on
                uint8_t cmd[] = {0x01, 0x20, 0x32};
                exchange(cmd, sizeof(cmd));
                assistOn = true;

                // TODO: Start waiting for MYSTERY BAT COMMAND 12 (with arg 1), while
                // doing handoffs. So this should be a state? And in the handoff we
                // signal the next state.
            } else {
                // Set assist level
                uint8_t cmd[] = {0x01, 0x21, 0x34, level};
                exchange(cmd, sizeof(cmd));

#if CONFIG_ION_CU2
                // Notify display update
                xEventGroupSetBits(controlEventGroup, DISPLAY_UPDATE_BIT); 
#endif
                state = MOTOR_ON;
                step = 0;
            }
        } else if(state == TURN_MOTOR_OFF) {
            if(assistOn) {
                // Assist off
                uint8_t cmd[] = {0x01, 0x20, 0x33};
                exchange(cmd, sizeof(cmd));
                assistOn = false;

                // TODO: Start waiting for MYSTERY BAT COMMAND 12 (with arg 0), while
                // doing handoffs. So this should be a state? And in the handoff we
                // signal the next state.
            } else {
                // Motor off
                uint8_t cmd[] = {0x01, 0x21, 0x31, 0x00};
                exchange(cmd, sizeof(cmd));
                // NOTE: XHP after this will stop responding to handoff messages after some time (and a put data message)
                // TODO: Maybe wait for cmd 11? (No cmd 11 from XHP?)
                state = MOTOR_OFF;
                step = 0;
            }
        } else if(state == MOTOR_OFF) {
            // TODO: Do we want this, which still does handoff messages, or do we want
            // to stop listening to motor? Normally motor is powered off and we keep
            // sending handoffs for a while..
            if(modeShortPress) {
                // TODO: Previously I made the short press sticky, so we'd leave IDLE straight away
                // state = IDLE;
                // TODO: We're still chatting, so motor is responsive, but need to turn it back on since we did tell it to turn off..
                // Really still need to come up with a good setup here. Does the actual system even turn off the motor? Probably only after a while in '0' assist state.
                // And depending on wether we're moving (by motor update km/h messages)?
                state = TURN_MOTOR_ON;
#if CONFIG_ION_CU2
                step = 0;
#else
                step = 5;
#endif
            }
        }

        if(motorHandoffs) {
            if(!handoff()) {
                // Timeout, assume motor turned off.
#if CONFIG_ION_CU2
                stopButtonCheck();
#endif
                motorHandoffs = false;
                state = IDLE;
            }
        }
    }

    vTaskDelete(NULL);
}

extern "C" void app_main() {

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    controlEventGroup = xEventGroupCreate();

    xTaskCreatePinnedToCore(my_task, "my_task", 4096, NULL, 5, NULL, SECOND_CPU);

    // TODO: TO TASK
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = BIT64(LED_BUILTIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    blinkQueue = xQueueCreate(3, sizeof(blinkType));

    xTaskCreatePinnedToCore(blinkTask, "blinkTask", 2048, NULL, 5, NULL, FIRST_CPU);

#if CONFIG_ION_BUTTON
    bool held = false;
    button_event_t ev;
    QueueHandle_t button_events = pulled_button_init(BIT64(BUTTON) | BIT64(BUTTON_EXT), GPIO_PULLUP_ONLY);
    while(true) {
        if(xQueueReceive(button_events, &ev, 1000 / portTICK_PERIOD_MS)) {
            if((ev.pin == BUTTON || ev.pin == BUTTON_EXT) && (ev.event == BUTTON_UP)) {
                held = false;
                xEventGroupSetBits(controlEventGroup, BUTTON_MODE_SHORT_PRESS_BIT);
            }
            if(!held && (ev.pin == BUTTON || ev.pin == BUTTON_EXT) && (ev.event == BUTTON_HELD)) {
                xEventGroupSetBits(controlEventGroup, BUTTON_LIGHT_LONG_PRESS_BIT);
                held = true;
            }
        }
    }
#endif
}
