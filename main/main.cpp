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
#include "calibration.h"
#include "states/states.h"
#include "ctrl_event_group.h"
#include "msg_handling.h"

static const char *TAG = "app";

#define FIRST_CPU PRO_CPU_NUM

#if SOC_CPU_CORES_NUM > 1
    #define SECOND_CPU APP_CPU_NUM
#else
    #define SECOND_CPU PRO_CPU_NUM
#endif

#if CONFIG_ION_BUTTON
    #define BUTTON ((gpio_num_t)CONFIG_ION_BUTTON_BOARD_PIN)
    #define BUTTON_EXT ((gpio_num_t)CONFIG_ION_BUTTON_EXTERNAL_PIN)
#endif

#if CONFIG_ION_CHARGE
    #define CHARGE_PIN ((gpio_num_t)CONFIG_ION_CHARGE_PIN)
#endif

static TimerHandle_t measureBatTimer;

static void measureBatTimerCallback(TimerHandle_t xTimer) {
    setControlBits(MEASURE_BAT_BIT);
}

#if CONFIG_ION_KEEPALIVE
volatile bool myTaskAlive = false;
TimerHandle_t healthCheckTimer ;

static void checkMyTaskHealth(TimerHandle_t xTimer) {
    if (!myTaskAlive) {
        saveDistances();
        esp_restart();
    }
    myTaskAlive = false;  // Reset voor volgende check
}
#endif

static void doHandoff(ion_state * state) {
#if CONFIG_ION_CU3
    uint8_t handoffTarget = MSG_DISPLAY;
#else
    uint8_t handoffTarget = MSG_MOTOR;
#endif
    writeMessage(handoffMsg(handoffTarget));

    while(true) {
        // Keep handling responses, and subsequent incoming messages, until someone hands off back to us.
        bool sawValidMessage = false;
        messageType message = {};
        readResult readResult;
        do {
            // Replies to handoff should be a lot quicker then 250ms
            readResult = readMessage(&message, 250 / portTICK_PERIOD_MS);
            if(readResult == MSG_OK &&
                (message.source == handoffTarget ||
                (message.type == MSG_HANDOFF && message.target != handoffTarget))) {
                // We saw a good message from our target, or another handoff message (to another target). So probably it accepted the handoff.
                sawValidMessage = true;
            }
            if(readResult == MSG_TIMEOUT) {
                // The bus was quiet too long after our handoff message, or any subsequent messages.
                // The target is likely not listening or turned off.
                // E.g. CU3 removed, or XHP motor turned off (Toprun motor seems to stay chatty even when 'off').
                // Or some messages got mangled/lost somehow (probably seen as CRC error) and now everyone is waiting.

                if(handoffTarget == MSG_DISPLAY && !sawValidMessage) {
                    // Let's assume the CU3 display is removed, handoff to motor instead.
                    // TODO: We should remember this, and ping the display now and then to see if it's back.
                    handoffTarget = MSG_MOTOR;
                    writeMessage(handoffMsg(handoffTarget));
                    continue; // I'd prefer to jump to the outer loop, but this is good enough..
                }

#if CONFIG_ION_CU2
                stopButtonCheck();
#endif
                if(state->state == MOTOR_OFF) {
                    state->doHandoffs = false;
                    toIdleState(state);
                }
                return;
            }

            // We will ignore:
            // - Messages with CRC error
            // - Wakeup messages (we are already awake)
            // - Message not addressed to us
        } while(readResult != MSG_OK || message.target != MSG_BMS);

        // A message was sent to us, deal with it.
        messageHandlingResult handleResult = handleMessage(message, state);
        if(handleResult == CONTROL_TO_US) {
            // There was a handoff to us, so we're back in control. Exit the loop.
            return;
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

    initUart();

    loadDistances();

#if CONFIG_ION_CU2
    initCu2();
#endif

    initDisplay();
    initMotor();

    measureBatTimer = xTimerCreate("measureBatTimer", (100 / portTICK_PERIOD_MS), pdTRUE, (void *)0, measureBatTimerCallback);
    xTimerStart(measureBatTimer, 0);
	
#if CONFIG_ION_KEEPALIVE
    healthCheckTimer = xTimerCreate("healthCheckTimer", 60000 / portTICK_PERIOD_MS, pdTRUE, NULL, checkMyTaskHealth);
    xTimerStart(healthCheckTimer, 0);
#endif

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

#if CONFIG_ION_KEEPALIVE
        myTaskAlive = true;  // sign of life
#endif

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

        EventBits_t buttonBits = waitControlBits(BUTTON_MODE_SHORT_PRESS_BIT | BUTTON_MODE_LONG_PRESS_BIT | BUTTON_LIGHT_SHORT_PRESS_BIT | BUTTON_LIGHT_LONG_PRESS_BIT | WAKEUP_BIT | CALIBRATE_BIT, true, false, 0);
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

        if(modeLongPress) {
            resetTrip1(0);
            requestDisplayUpdate();
        }

#if CONFIG_ION_ADC
        EventBits_t bitsToCheck = MEASURE_BAT_BIT;
        EventBits_t bits = waitControlBits(bitsToCheck, false, false, 0);
        if((bits & MEASURE_BAT_BIT) != 0) {
            clearControlBits(MEASURE_BAT_BIT);
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
            doHandoff(&state);
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

    initControlEventGroup();

    xTaskCreatePinnedToCore(my_task, "my_task", 4096 * 2, NULL, 5, NULL, SECOND_CPU);

#if CONFIG_ION_BUTTON
    bool held = false;
    button_event_t ev;
    QueueHandle_t button_events = pulled_button_init(BIT64(BUTTON) | BIT64(BUTTON_EXT), GPIO_PULLUP_ONLY);
    while(true) {
        if(xQueueReceive(button_events, &ev, 1000 / portTICK_PERIOD_MS)) {
            if((ev.pin == BUTTON || ev.pin == BUTTON_EXT) && (ev.event == BUTTON_UP)) {
                held = false;
                setControlBits(BUTTON_MODE_SHORT_PRESS_BIT);
            }
            if(!held && (ev.pin == BUTTON || ev.pin == BUTTON_EXT) && (ev.event == BUTTON_HELD)) {
                setControlBits(BUTTON_LIGHT_LONG_PRESS_BIT);
                held = true;
            }
        }
    }
#endif
}
