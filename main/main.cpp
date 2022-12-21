#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/unistd.h>
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "bytes.h"
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
#include "display.h"
#include "motor.h"
#include "relays.h"
#include "trip.h"
#include "states/states.h"
#include "storage.h"

static const char *TAG = "app";

#define FIRST_CPU PRO_CPU_NUM

#if SOC_CPU_CORES_NUM > 1
    #define SECOND_CPU APP_CPU_NUM
#else
    #define SECOND_CPU PRO_CPU_NUM
#endif

#if CONFIG_ION_BUTTON
    #define BUTTON (GPIO_NUM_0)
    #define BUTTON_EXT (GPIO_NUM_4)
#endif

#if CONFIG_ION_CHARGE
    #define CHARGE_PIN ((gpio_num_t)CONFIG_ION_CHARGE_PIN)
#endif

static const int BUTTON_MODE_SHORT_PRESS_BIT = BIT0;
static const int BUTTON_MODE_LONG_PRESS_BIT = BIT1;
static const int BUTTON_LIGHT_SHORT_PRESS_BIT = BIT2;
static const int BUTTON_LIGHT_LONG_PRESS_BIT = BIT3;
static const int IGNORE_HELD_BIT = BIT4;
static const int WAKEUP_BIT = BIT5;
static const int CALIBRATE_BIT = BIT6;
static const int MEASURE_BAT_BIT = BIT7;

static EventGroupHandle_t controlEventGroup;

enum messageHandlingResult {
    // We got a handoff back, so we get to send the next message
    CONTROL_TO_US,
    // We had to reply to a message, so sender is next to send a message.
    CONTROL_TO_SENDER,
    // We did not get a timely reply to a handoff message.
    HANDOFF_TIMEOUT
};

static TimerHandle_t measureBatTimer;

static void measureBatTimerCallback(TimerHandle_t xTimer) { xEventGroupSetBits(controlEventGroup, MEASURE_BAT_BIT); }

static messageHandlingResult handleMotorMessage(ion_state * state) {
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
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 0 && message.command == 0x01) {
        // MYSTERY BATTERY COMMAND 01
        uint8_t payload[] = {0x02, 0x02};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 0 && message.command == CMD_BAT_STATUS_MOTOR_OFF) {
        state->motorOffAck = true;
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 1 && message.command == CMD_BAT_STATUS_ASSIST) {
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 0 && message.command == CMD_BAT_WAKEUP) {
        xEventGroupSetBits(controlEventGroup, WAKEUP_BIT);                     
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 1 && message.command == CMD_BAT_CALIBRATE) {
        xEventGroupSetBits(controlEventGroup, CALIBRATE_BIT);                     
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 1 && message.command == CMD_BAT_SET_LIGHT) {
        setLight(message.payload[0]);
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 1 && message.command == CMD_BAT_SET_ASSIST_LEVEL) {
        state->level = message.payload[0];
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_SENDER;
#if CONFIG_ION_CU3
    } else if(handleCu3Message(message)) {
        return CONTROL_TO_SENDER;
#endif
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 2 && message.command == CMD_GET_DATA && message.payload[1] == 0x2a) {
        // GET DATA 002a 00:2a(Unknown)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], 0x01};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 4 && message.command == CMD_GET_DATA && message.payload[1] == 0x38 && message.payload[3] == 0x3a) {
        // GET DATA 9438283a 14:38(Calibration A) 28:3a(Calibration B)
        uint8_t payload[] = {0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // Data (last 10 bytes) to be replaced

        if(calibrationFileExists()) {
            if(!readCalibrationData(payload + 1)) {
                return CONTROL_TO_SENDER;
            }
        } else {
            // Backup data
            // Gold small test: 94 38 4b 13 28 3a 3e 98 ed f3
            uint8_t data[] = {0x94, 0x38, 0x4b, 0x15, 0x28, 0x3a, 0x3e, 0x91, 0x79, 0x50}; // This needs to be good calibration data!
            memcpy(payload + 1, data, 10);
        }

        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 10 && message.command == CMD_PUT_DATA && message.payload[1] == 0xc0 && message.payload[5] == 0xc1) {
        // PUT DATA c0/c1
        state->speed = toUint16(message.payload, 2);
        distanceUpdate(toUint32(message.payload, 6));

        uint8_t payload[] = {0x00};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        requestDisplayUpdate();
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 10 && message.command == CMD_PUT_DATA && message.payload[1] == 0x38 && message.payload[5] == 0x3a) {
        // PUT DATA 38/3a
        if(!writeCalibrationData(message.payload)) {
            return CONTROL_TO_SENDER;
        }

        uint8_t payload[] = {0x00};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_SENDER;
    }

    ESP_LOGI(TAG, "Unexpected: Tgt:%d, Src:%d, Type:%d, Command:%d", message.target, message.source, message.type, message.command);
    ESP_LOG_BUFFER_HEX(TAG, message.payload, message.payloadSize);

    return CONTROL_TO_SENDER;
}

static bool handoff(ion_state * state) {

#if CONFIG_ION_CU3
    uint8_t handoffTarget = MSG_DISPLAY;
#else
    uint8_t handoffTarget = MSG_MOTOR;
#endif
    writeMessage(handoffMsg(handoffTarget));

    while(true) {
        messageHandlingResult result = handleMotorMessage(state);
        if(result == CONTROL_TO_US) {
            return true;
        }
        if(result == HANDOFF_TIMEOUT) {
            return false;
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

    initRelay();

    initLight();

#if CONFIG_ION_CHARGE
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = BIT64(CHARGE_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
#endif

    initBlink();

#if CONFIG_ION_ADC
    adc_init();
#endif

    init_spiffs();

    initUart();

    loadDistances();

#if CONFIG_ION_CU2
    initCu2(controlEventGroup, 
            BUTTON_MODE_SHORT_PRESS_BIT, 
            BUTTON_MODE_LONG_PRESS_BIT, 
            BUTTON_LIGHT_SHORT_PRESS_BIT, 
            BUTTON_LIGHT_LONG_PRESS_BIT, 
            IGNORE_HELD_BIT
    );
#endif

    initDisplay();
    initMotor();

    measureBatTimer = xTimerCreate("measureBatTimer", (100 / portTICK_PERIOD_MS), pdTRUE, (void *)0, measureBatTimerCallback);

    xTimerStart(measureBatTimer, 0);

    ion_state state = {
        .state = IDLE,
        .step = 0,
        .displayOn = false,
        .assistOn = false,
        .levelSet = 0,
        .doHandoffs = false,
        .motorOffAck = false,
        .level = 0,
        .speed = 0
    };

    while(true) {

        // TODO:
        // More use of timeouts
        // See if we really need 8k stack (copying message structure a lot I guess)

        // TODO:
        // Instead .. ? can we wait on event bits AND rx?
        // Serial should lead, buttons are uncommon.
        // Can we generate a eventgroup bit from uart?


#if CONFIG_ION_CHARGE
        // Charge pin is pulled to ground to activate.
        const bool chargePin = gpio_get_level(CHARGE_PIN) == 0;
        if(chargePin && state.state != CHARGING) {
            toChargingState(&state);
        }
#endif

        EventBits_t buttonBits = xEventGroupWaitBits(controlEventGroup, BUTTON_MODE_SHORT_PRESS_BIT | BUTTON_MODE_LONG_PRESS_BIT | BUTTON_LIGHT_SHORT_PRESS_BIT | BUTTON_LIGHT_LONG_PRESS_BIT | WAKEUP_BIT | CALIBRATE_BIT, true, false, 0);
        const bool modeShortPress = (buttonBits & BUTTON_MODE_SHORT_PRESS_BIT) != 0;
        const bool modeLongPress = (buttonBits & BUTTON_MODE_LONG_PRESS_BIT) != 0;
        const bool lightShortPress = (buttonBits & BUTTON_LIGHT_SHORT_PRESS_BIT) != 0;
        const bool lightLongPress = (buttonBits & BUTTON_LIGHT_LONG_PRESS_BIT) != 0;
        const bool wakeup = (buttonBits & WAKEUP_BIT) != 0;
        const bool calibrate = (buttonBits & CALIBRATE_BIT) != 0;

        if(lightShortPress) {
            toggleLight();
            requestDisplayUpdate();
        }


#if CONFIG_ION_ADC
        EventBits_t bitsToCheck = MEASURE_BAT_BIT;
        EventBits_t bits = xEventGroupWaitBits(controlEventGroup, bitsToCheck, false, false, 0);
        if((bits & MEASURE_BAT_BIT) != 0) {
            xEventGroupClearBits(controlEventGroup, MEASURE_BAT_BIT);
            measureBat();
        } else
#endif
        if(handleDisplayUpdate(&state)) {
        } else if(handleMotorUpdate()) {
        } else if(state.state == IDLE) {
            handleIdleState(&state, modeShortPress);
        } else if(state.state == TURN_MOTOR_ON) {
            handleTurnMotorOnState(&state);
        } else if(state.state == MOTOR_ON) {
            handleMotorOnState(&state, modeShortPress, lightLongPress, calibrate);
#if CONFIG_ION_CHARGE
        } else if(state.state == CHARGING)  {
            handleChargingState(&state, chargePin);
#endif
        } else if(state.state == START_CALIBRATE) {
            handleCalibrateState(&state);
        } else if(state.state == SET_ASSIST_LEVEL) {
            handleSetAssistLevelState(&state);       
        } else if(state.state == TURN_MOTOR_OFF) {
            handleTurnMotorOffState(&state);
        } else if(state.state == MOTOR_OFF) {
            handleMotorOffState(&state, modeShortPress, wakeup);
        }

        if(state.doHandoffs) {
            if(!handoff(&state)) {
                // Timeout, assume motor turned off. CU3 will keep chatting, so no timeout then.
#if CONFIG_ION_CU2
                stopButtonCheck();
#endif
                state.doHandoffs = false;
                toIdleState(&state);
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
