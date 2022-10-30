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
#include "esp_timer.h"
#include "nvs_flash.h"
#include "button.h"
#include "bow.h"
#include "cmds.h"
#include "blink.h"
#if CONFIG_ION_CU2
    #include "cu2.h"
#elif CONFIG_ION_CU3
    #include "cu3.h"
#endif
#if CONFIG_ION_ADC
    #include "bat.h"
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

static const int BUTTON_MODE_SHORT_PRESS_BIT = BIT0;
static const int BUTTON_MODE_LONG_PRESS_BIT = BIT1;
static const int BUTTON_LIGHT_SHORT_PRESS_BIT = BIT2;
static const int BUTTON_LIGHT_LONG_PRESS_BIT = BIT3;
#if CONFIG_ION_CU2
static const int CHECK_BUTTON_BIT = BIT4;
#endif
#if CONFIG_ION_CU2 || CONFIG_ION_CU3
static const int DISPLAY_UPDATE_BIT = BIT5;
#endif
static const int IGNORE_HELD_BIT = BIT6;
static const int WAKEUP_BIT = BIT7;
static const int CALIBRATE_BIT = BIT8;
static const int MOTOR_UPDATE_BIT = BIT9;

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

static EventGroupHandle_t controlEventGroup;

enum control_state { IDLE, START_CALIBRATE, TURN_MOTOR_ON, MOTOR_ON, SET_ASSIST_LEVEL, TURN_MOTOR_OFF, MOTOR_OFF };

struct ion_state {
    // The state we're in
    control_state state;

    // The step of the state we're in, most states follow a sequence of commands
    uint8_t step;

    // Should the display be 'on' ('off' is logo for CU3).
    bool displayOn;

    // Is assist currently on
    bool assistOn;

    // The assist level currently active (set in the motor)
    uint8_t levelSet;

    // Whether we should do (regular) handoffs to other components
    bool doHandoffs;

};

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

// Time offset in seconds, add this to time since boot to get real time.
static uint32_t offset = 0;

// Light on/off
static bool lightOn = false;

// Motor indicates off is ready.
static bool motorOffAck = false;

static uint16_t batVal = 6000; // Bat. value for CU3, range is something like -10% - 100%
static uint16_t batMax = 11000; // The max value for batVal (100%)

static TimerHandle_t motorUpdateTimer;

static void setLight(bool value) {
    lightOn = value;
#if CONFIG_ION_LIGHT
    gpio_set_level(LIGHT_PIN, value ? LIGHT_ON : LIGHT_OFF);
#endif
}

static void toggleLight() {
    setLight(!lightOn);
}

static void initLight() {
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

#if CONFIG_ION_RELAY
static void setRelay(bool value) {
    gpio_set_level(RELAY_PIN, value ? RELAY_ON : RELAY_OFF);
}

static void initRelay() {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = BIT64(RELAY_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    setRelay(false);
}
#endif

static uint16_t toUint16(uint8_t *buffer, size_t offset) { 
    return ((uint16_t)buffer[offset] << 8) | ((uint16_t)buffer[offset + 1] << 0); 
}

static uint32_t toUint32(uint8_t *buffer, size_t offset) {
    return ((uint32_t)buffer[offset] << 24) | ((uint32_t)buffer[offset + 1] << 16) | ((uint32_t)buffer[offset + 2] << 8) | ((uint32_t)buffer[offset + 3] << 0);
}

static void motorUpdateTimerCallback(TimerHandle_t xTimer) { xEventGroupSetBits(controlEventGroup, MOTOR_UPDATE_BIT); }

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
    } while(result != MSG_OK || message.target != MSG_BMS);

    if(message.type == MSG_HANDOFF) {
        // Handoff back to us
        return CONTROL_TO_US;
    } else if(message.type == MSG_PING_REQ) {
        // ESP_LOGI(TAG, "|PING");
        writeMessage(pingResp(message.source, MSG_BMS));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 0 && message.command == 0x01) {
        // MYSTERY BATTERY COMMAND 01
        uint8_t payload[] = {0x02, 0x02};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 0 && message.command == CMD_BAT_STATUS_MOTOR_OFF) {
        motorOffAck = true;
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 1 && message.command == CMD_BAT_STATUS_ASSIST) {
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 0 && message.command == CMD_BAT_WAKEUP) {
        xEventGroupSetBits(controlEventGroup, WAKEUP_BIT);                     
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 1 && message.command == CMD_BAT_CALIBRATE) {
        xEventGroupSetBits(controlEventGroup, CALIBRATE_BIT);                     
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 1 && message.command == CMD_BAT_SET_LIGHT) {
        setLight(message.payload[0]);
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 1 && message.command == CMD_BAT_SET_ASSIST_LEVEL) {
        level = message.payload[0];
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 2 && message.command == CMD_GET_DATA && message.payload[1] == 0x18) {
        // GET DATA 1418 14:18(Battery level)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], (uint8_t)(batVal >> 8), (uint8_t)(batVal >> 0)};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 4 && message.command == CMD_GET_DATA && message.payload[1] == 0x18 && message.payload[3] == 0x1a) {
        // GET DATA 9418141a 14:18(Battery level) 14:1a(Max battery level)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], (uint8_t)(batVal >> 8), (uint8_t)(batVal >> 0), message.payload[2], message.payload[3], (uint8_t)(batMax >> 8), (uint8_t)(batMax >> 0)};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 2 && message.command == CMD_GET_DATA && message.payload[1] == 0x2a) {
        // GET DATA 002a 00:2a(Unknown)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], 0x01};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 4 && message.command == CMD_GET_DATA && message.payload[1] == 0x38 && message.payload[3] == 0x3a) {
        // GET DATA 9438283a 14:38(Calibration A) 28:3a(Calibration B)
        uint8_t payload[] = {0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // Data (last 10 bytes) to be replaced

        struct stat st;
        if(stat(CALIBRATION_FILE, &st) == 0) {
            FILE *fp = fopen(CALIBRATION_FILE, "r");
            if(fp == NULL) {
                ESP_LOGE(TAG, "Failed to open calibration file for reading");
                return CONTROL_TO_MOTOR;
            }
            fread(payload + 1, 1, 10, fp);
            fclose(fp);
        } else {
            // Backup data
            // Gold small test: 94 38 4b 13 28 3a 3e 98 ed f3
            uint8_t data[] = {0x94, 0x38, 0x4b, 0x15, 0x28, 0x3a, 0x3e, 0x91, 0x79, 0x50}; // This needs to be good calibration data!
            memcpy(payload + 1, data, 10);
        }

        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 2 && message.command == CMD_GET_DATA && message.payload[1] == 0x3b) {
        // GET DATA 083b 08:3b(Distance to maintenance)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], 0x00, 0x01, 0xE2, 0x08};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 2 && message.command == CMD_GET_DATA && message.payload[1] == 0x80) {
        // GET DATA 0880 08:80(Total distance)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], 0x00, 0x11, 0xd4, 0xcd};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 2 && message.command == CMD_GET_DATA && message.payload[1] == 0x8e) {
        // GET DATA 088e 08:8e(Time)

        int64_t seconds = esp_timer_get_time() / (1000 * 1000) + offset;
        uint8_t byte0 = (seconds >> 24) & 0xff;
        uint8_t byte1 = (seconds >> 16) & 0xff; 
        uint8_t byte2 = (seconds >> 8) & 0xff;
        uint8_t byte3 = seconds & 0xff;

        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], byte0, byte1, byte2, byte3};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 2 && message.command == CMD_GET_DATA && message.payload[1] == 0x94) {
        // GET DATA 2894 28:94(Unknown)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], 0x40, 0x0e, 0x14, 0x7b};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 3 && message.command == CMD_GET_DATA && message.payload[1] == 0x99 && message.payload[2] == 0x00) {
        // GET DATA 489900 48:99[0](Trip time)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf6};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 3 && message.command == CMD_GET_DATA && message.payload[1] == 0x9a && message.payload[2] == 0x00) {
        // GET DATA 449a00 44:9a[0](Max speed)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], 0x02, 0x00, 0x00, 0x00, 0xd0};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 10 && message.command == CMD_PUT_DATA && message.payload[1] == 0xc0 && message.payload[5] == 0xc1) {
        // PUT DATA c0/c1
        speed = toUint16(message.payload, 2);
        trip = toUint32(message.payload, 6);

        uint8_t payload[] = {0x00};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
#if CONFIG_ION_CU2 || CONFIG_ION_CU3
        // Notify display update
        xEventGroupSetBits(controlEventGroup, DISPLAY_UPDATE_BIT);
#endif
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 10 && message.command == CMD_PUT_DATA && message.payload[1] == 0x38 && message.payload[5] == 0x3a) {
        // PUT DATA 38/3a
        FILE *fp = fopen(CALIBRATION_FILE, "w");
        if(fp == NULL) {
            ESP_LOGE(TAG, "Failed to open calibration file for writing");
            return CONTROL_TO_MOTOR;
        }
        fwrite(message.payload, 1, 10, fp);
        fclose(fp);

        uint8_t payload[] = {0x00};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_MOTOR;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 6 && message.command == CMD_PUT_DATA && message.payload[1] == 0x8e) {
        uint32_t newTime = toUint32(message.payload, 2);
        int64_t seconds = esp_timer_get_time() / (1000 * 1000);
        offset = newTime - seconds;

        uint8_t payload[] = {0x00};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_MOTOR;
    }

    ESP_LOGI(TAG, "Unexpected: Tgt:%d, Src:%d, Type:%d, Command:%d", message.target, message.source, message.type, message.command);
    ESP_LOG_BUFFER_HEX(TAG, message.payload, message.payloadSize);

    return CONTROL_TO_MOTOR;
}

static bool handoff() {

#if CONFIG_ION_CU3
    uint8_t handoffTarget = MSG_DISPLAY;
#else
    uint8_t handoffTarget = MSG_MOTOR;
#endif
    writeMessage(handoffMsg(handoffTarget));

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

static void toTurnMotorOnState(ion_state * state) {
    state->displayOn = true;
    queueBlink(1, 500, 50);
#if CONFIG_ION_RELAY
    setRelay(true);
#endif

    state->state = TURN_MOTOR_ON;
    state->step = 0;
}

static void toMotorOnState(ion_state * state) {
    state->state = MOTOR_ON;
    state->step = 0;
}

static void toCalibrateState(ion_state * state) {
    queueBlink(10, 100, 100);

    state->state = START_CALIBRATE;
    state->step = 0;
}

static void toSetAssistLevelState(ion_state * state) {
    if(level == 0) {
        queueBlink(2, 250, 50);
    } else {
        queueBlink(level, 100, 50);
    }

    state->state = SET_ASSIST_LEVEL;
    state->step = 0;
}

static void toTurnMotorOffState(ion_state * state) {
    state->displayOn = false;
    queueBlink(2, 400, 50);

    state->state = TURN_MOTOR_OFF;
    state->step = 0;
}

/**
 * Put data from bat. to motor. Sent initially, and then every 10, 15, 50 seconds? (no good recording with timging yet).
 * Values:
 * b0 - Almost always 2500, only seen it lower on low bat uphill.
 * b1 - Volts in 100mv Goes up and down, so likely current voltage, even under load.
 */
static void motorUpdate() {
    uint16_t unknown = 2500; // Normally 2500, very sometimes much lower, on low battery up hill?
    uint16_t volts = 276; // Volts, in 100mv

    uint8_t payload[] = {
            0x94, 0xb0, 
            (uint8_t)(unknown >> 8), (uint8_t)(unknown >> 0), 
            0x14, 0xb1, 
            (uint8_t)(volts >> 8), (uint8_t)(volts >> 0)};
    exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_PUT_DATA, payload, sizeof(payload)));
}

static void handleTurnMotorOnState(ion_state * state) {

    static uint8_t displaySerial[8] = {};
    static uint8_t motorSlot2Serial[8] = {};

#if CONFIG_ION_CU3
    const uint8_t nextStep = 1;
    // TODO: Update display every 1.5 second, unless already updated (from motor message)
    if(state->step == 0) {
        displayUpdateCu3(DSP_SCREEN, state->displayOn, true, false, 0, 0, 0, 0);
    } else 
#elif CONFIG_ION_CU2
    const uint8_t nextStep = 5;
    if(state->step == 0) {
        // Button check command with a special value, maybe just resets
        // default/display? Or sets timeout? Or initializes display 'clock'?
        uint8_t payload[] = {0x80};
        messageType message = {};
        readResult result = exchange(cmdReq(MSG_DISPLAY, MSG_BMS, CMD_BUTTON_POLL, payload, sizeof(payload)), &message, 225 / portTICK_PERIOD_MS );
    } else if(state->step == 1) {
        // Update display
        displayUpdate(false, ASS_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_SOLID, BLNK_SOLID, true, 25, 0xccc, 0xccccc);
    } else if(state->step == 2) {
        // Unknown command which is always the same and always sent to the
        // display at this point.
        uint8_t payload[] = {0x04, 0x08};
        exchange(cmdReq(MSG_DISPLAY, MSG_BMS, 0x25, payload, sizeof(payload)));
    } else if(state->step == 3) {
        // First normal button check command, after this should run every 100ms.
        buttonCheck();
        startButtonCheck();
    } else if(state->step == 4) {
        // Set default display, which is shown if the display isn't updated for a bit (?)
        displayUpdate(true, ASS_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, false, 10, 0xccc, 0xccccc);
    } else 
#else
    const uint8_t nextStep = 0;
#endif
    if(state->step == nextStep) {
        messageType message = {};
        // Motor on
        // Original BMS seems to repeat handoff till the motor responds, with 41ms between commands, but this should also work.
        exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_MOTOR_ON), &message, 41 / portTICK_PERIOD_MS);
        state->doHandoffs = true;
    } else if(state->step == nextStep + 1) {
        motorUpdate();
        xTimerStart(motorUpdateTimer, 0);
#if CONFIG_ION_CU2 || CONFIG_ION_CU3
    } else if(state->step == nextStep + 2) {
        messageType response = {};
        exchange(cmdReq(MSG_DISPLAY, MSG_BMS, CMD_GET_SERIAL), &response);
        memcpy(displaySerial, response.payload, 8);
    } else if(state->step == nextStep + 3) {
        // Get serial progammed in motor slot 2
        messageType response = {};
        uint8_t payload[] = {0x40, 0x5c, 0x00};
        exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_GET_DATA, payload, sizeof(payload)), &response);
        if(response.payload[3] == 8) {
            memcpy(motorSlot2Serial, response.payload + 4, 8);
        }
        if(memcmp(displaySerial, motorSlot2Serial, 8) == 0) {
            toMotorOnState(state);
            return;
        }
    } else if(state->step == nextStep + 4) {
        // Program serial in motor slot 2
        uint8_t payload[] = {0x40, 0x5c, 0x00, 0x08, 0x08, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
        memcpy(payload + 5, displaySerial, 8);
        messageType response = {};
        exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_PUT_DATA, payload, sizeof(payload)), &response);
#endif
        toMotorOnState(state);
        return;
    }
    state->step++;
}

static void handleMotorOnState(ion_state * state, bool modeShortPress, bool lightLongPress, bool calibrate) {

    static int64_t lastMoving = 0;
    int64_t now = esp_timer_get_time();

    if(state->step == 0 || speed > 0 || state->levelSet != 0) {
        lastMoving = now;
    }

    if(state->step == 0) {
        state->step++;
    }

    if(now -lastMoving > 10 * 1000 * 1000 ) {
        toTurnMotorOffState(state);
        return;
    }

    // Handle calibration 'request' from a CU2 display, holding the light button while level is 0 and light is off.
    if((level == 0x00 && lightOn == false && lightLongPress) || calibrate) {
        toCalibrateState(state);
        return;
    } 

    // Handle level change request from CU2 display, by pressing mode button.
    if(modeShortPress) {
        level = (level + 1) % 4;
    }

    if(level != state->levelSet) {
        toSetAssistLevelState(state);
        return;
    }            
}

static void handleStartCalibrateState(ion_state * state) {
    // TODO: Turn motor (power) on if it's off (and wait for reply? can we see that in log handoffs?)
    // - handoffs dp/bat, DP> light on, or assist level, or calibrate
    //   100 handoffs (200 msg) later: 
    //   >> Light on: put data BT>MT b0/b1
    //   almost directly after motor turn on
    //   >> cal: almost directly after cal cmd (35)
    //   XXX handoffs later DP(CU3) pings motor, and starts to include it in handoffs
    //   Motor does get data 2a
    if(state->step == 0) {
        exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_CALIBRATE));
    } else if (state->step == 1) {
        // Get data, which is common after calibrate. No idea what it's for.
        uint8_t payload[] = {0x00, 0xdf};
        exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_GET_DATA, payload, sizeof(payload)));
#if CONFIG_ION_CU3
    } else if (state->step == 2) {
        // Let the display know calibration is done, not sure about what the payload means.
        uint8_t payload[] = {0x01, 0x01};
        exchange(cmdReq(MSG_DISPLAY, MSG_BMS, 0x2a, payload, sizeof(payload)));
#endif
        // BMS actually stops listening here, it ignores (some?) motor messages.
        toMotorOnState(state);
        return;
    }
    state->step++;
}

static void handleSetAssistLevelState(ion_state * state) {
    if(level == 0) {
        if(state->assistOn) {
            exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_ASSIST_OFF));
            state->assistOn = false;

            // TODO: Start waiting for MYSTERY BAT COMMAND 12 (with arg 0), while
            // doing handoffs. So this should be a state? And in the handoff we
            // signal the next state.
            state->levelSet = level;
#if CONFIG_ION_CU2 || CONFIG_ION_CU3
            // Notify display update
            xEventGroupSetBits(controlEventGroup, DISPLAY_UPDATE_BIT);
#endif
        }
        toMotorOnState(state);
    } else {
        if(!state->assistOn) {
            exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_ASSIST_ON));
            state->assistOn = true;

            // TODO: Start waiting for MYSTERY BAT COMMAND 12 (with arg 1), while
            // doing handoffs. So this should be a state? And in the handoff we
            // signal the next state.
        } else {
            uint8_t payload[] = {level};
            exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_SET_ASSIST_LEVEL, payload, sizeof(payload)));

            state->levelSet = level;
#if CONFIG_ION_CU2 || CONFIG_ION_CU3
            // Notify display update
            xEventGroupSetBits(controlEventGroup, DISPLAY_UPDATE_BIT);
#endif
        toMotorOnState(state);
        }
    }     
}

static void handleTurnMotorOffState(ion_state * state) {
    if(state->assistOn) {
        exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_ASSIST_OFF));
        state->assistOn = false;

        // TODO: Start waiting for MYSTERY BAT COMMAND 12 (with arg 0), while
        // doing handoffs. So this should be a state? And in the handoff we
        // signal the next state.
    } else {
        if(state->step == 0) {
            xTimerStop(motorUpdateTimer, 0);

            motorOffAck = false;
            uint8_t payload[] = {0x00};
            exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_MOTOR_OFF, payload, sizeof(payload)));
            // NOTE: XHP after this will stop responding to handoff messages after some time (and a put data message)

            state->step++;
        } else if (state->step == 1)
            if(motorOffAck) {
#if CONFIG_ION_RELAY
                setRelay(false);
#endif

                queueBlink(4, 100, 300);
                state->state = MOTOR_OFF;
                state->step = 0;
            }
    }
}

static void my_task(void *pvParameter) {

#if CONFIG_ION_RELAY
    initRelay();
#endif

    initLight();

    initBlink();

#if CONFIG_ION_ADC
    adc_init();
#endif

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

    motorUpdateTimer = xTimerCreate("motorUpdateTimer", (10000 / portTICK_PERIOD_MS), pdTRUE, (void *)0, motorUpdateTimerCallback);

    ion_state state = {
        .state = IDLE,
        .step = 0,
        .displayOn = false,
        .assistOn = false,
        .levelSet = level,
        .doHandoffs = false
    };

    while(true) {

        // TODO:
        // More use of timeouts
        // See if we really need 8k stack (copying message structure a lot I guess)

        // TODO:
        // Instead .. ? can we wait on event bits AND rx?
        // Serial should lead, buttons are uncommon.
        // Can we generate a eventgroup bit from uart?

        EventBits_t buttonBits = xEventGroupWaitBits(controlEventGroup, BUTTON_MODE_SHORT_PRESS_BIT | BUTTON_MODE_LONG_PRESS_BIT | BUTTON_LIGHT_SHORT_PRESS_BIT | BUTTON_LIGHT_LONG_PRESS_BIT | WAKEUP_BIT | CALIBRATE_BIT, true, false, 0);
        const bool modeShortPress = (buttonBits & BUTTON_MODE_SHORT_PRESS_BIT) != 0;
        const bool modeLongPress = (buttonBits & BUTTON_MODE_LONG_PRESS_BIT) != 0;
        const bool lightShortPress = (buttonBits & BUTTON_LIGHT_SHORT_PRESS_BIT) != 0;
        const bool lightLongPress = (buttonBits & BUTTON_LIGHT_LONG_PRESS_BIT) != 0;
        const bool wakeup = (buttonBits & WAKEUP_BIT) != 0;
        const bool calibrate = (buttonBits & CALIBRATE_BIT) != 0;

#if CONFIG_ION_ADC
        // CU3 seems to calculate % with something close to:
        // floor( (val - (0.091 * max)) / 0.009 * max )
        // Below we do the reverse.
        uint32_t offsetK = (91 * batMax);
        uint32_t onePercentK = (9 * batMax);
        uint32_t valueK = offsetK + onePercentK * adc_measure() + onePercentK / 2;
        batVal = (uint16_t) (valueK / 1000);
#endif

        if(lightShortPress) {
            toggleLight();
#if CONFIG_ION_CU2 || CONFIG_ION_CU3
            xEventGroupSetBits(controlEventGroup, DISPLAY_UPDATE_BIT); 
#endif
        }

#if CONFIG_ION_CU2
        EventBits_t bits = xEventGroupWaitBits(controlEventGroup, CHECK_BUTTON_BIT | DISPLAY_UPDATE_BIT | MOTOR_UPDATE_BIT, false, false, 0);
        if((bits & CHECK_BUTTON_BIT) != 0) {
            xEventGroupClearBits(controlEventGroup, CHECK_BUTTON_BIT);
            buttonCheck();
        } else
#elif CONFIG_ION_CU3
        EventBits_t bits = xEventGroupWaitBits(controlEventGroup, DISPLAY_UPDATE_BIT | MOTOR_UPDATE_BIT, false, false, 0);
#endif
#if CONFIG_ION_CU2 || CONFIG_ION_CU3
         if((bits & DISPLAY_UPDATE_BIT) != 0) {
            xEventGroupClearBits(controlEventGroup, DISPLAY_UPDATE_BIT);
#if CONFIG_ION_CU2
            showState(level, lightOn, speed, trip);
#elif CONFIG_ION_CU3
            showStateCu3(level, state.displayOn, lightOn, speed, trip);
#endif
        } else 
#endif
        if((bits & MOTOR_UPDATE_BIT) != 0) {
            xEventGroupClearBits(controlEventGroup, MOTOR_UPDATE_BIT);
            motorUpdate();
        } else if(state.state == IDLE) {
            if(modeShortPress) {
                toTurnMotorOnState(&state);
            } else {
                messageType message = {};
                readResult result = readMessage(&message, 50 / portTICK_PERIOD_MS );
                if(result == MSG_WAKEUP) {
                    ESP_LOGI(TAG, "Wakeup!");
                    xEventGroupSetBits(controlEventGroup, IGNORE_HELD_BIT); 
                    toTurnMotorOnState(&state);
                } else if(result == MSG_OK){
                    ESP_LOGI(TAG, "Incoming: Tgt:%d, Src:%d, Type:%d, Command:%d", message.target, message.source, message.type, message.command);
                    ESP_LOG_BUFFER_HEX(TAG, message.payload, message.payloadSize);
                }
            }
        } else if(state.state == TURN_MOTOR_ON) {
            handleTurnMotorOnState(&state);
        } else if(state.state == MOTOR_ON) {
            handleMotorOnState(&state, modeShortPress, lightLongPress, calibrate);
        } else if(state.state == START_CALIBRATE) {
            handleStartCalibrateState(&state);
        } else if(state.state == SET_ASSIST_LEVEL) {
            handleSetAssistLevelState(&state);       
        } else if(state.state == TURN_MOTOR_OFF) {
            handleTurnMotorOffState(&state);
        } else if(state.state == MOTOR_OFF) {
            // Motor is off, but we may still get handoff messages from it, or the display (CU3).
            if(modeShortPress || wakeup) {
                toTurnMotorOnState(&state);
            }
        }

        if(state.doHandoffs) {
            if(!handoff()) {
                // Timeout, assume motor turned off. CU3 will keep chatting, so no timeout then.
#if CONFIG_ION_CU2
                stopButtonCheck();
#endif
                state.doHandoffs = false;
                state.state = IDLE;
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

    xTaskCreatePinnedToCore(my_task, "my_task", 4096 * 2, NULL, 5, NULL, SECOND_CPU);

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
