#include <sys/unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "bytes.h"
#include "bat.h"
#include "bow.h"
#include "cmds.h"
#include "motor.h"

static EventGroupHandle_t eventGroupHandle;
static TimerHandle_t motorUpdateTimer;

static const int MOTOR_UPDATE_BIT = BIT0;

static void motorUpdateTimerCallback(TimerHandle_t xTimer) { xEventGroupSetBits(eventGroupHandle, MOTOR_UPDATE_BIT); }

void initMotor() {
    eventGroupHandle = xEventGroupCreate();
    motorUpdateTimer = xTimerCreate("motorUpdateTimer", (10000 / portTICK_PERIOD_MS), pdTRUE, (void *)0, motorUpdateTimerCallback);
}

/**
 * Put data from bat. to motor. Sent initially, and then every 10, 15, 50 seconds? (no good recording with timging yet).
 * Values:
 * b0 - Almost always 2500, only seen it lower on low bat uphill.
 * b1 - Volts in 100mv Goes up and down, so likely current voltage, even under load.
 */
void motorUpdate() {
    uint16_t unknown = 2500; // Normally 2500, very sometimes much lower, on low battery up hill? Amp limit in 10ma??
    uint16_t volts = getBatMv() / 100; // Volts, in 100mv

    uint8_t payload[] = {
            0x94, 0xb0,
            FROM_UINT16(unknown), 
            0x14, 0xb1,
            FROM_UINT16(volts)};
    exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_PUT_DATA, payload, sizeof(payload)));
}

void startMotorUpdates() {
    xTimerStart(motorUpdateTimer, 0);
}

void stopMotorUpdates() {
    xTimerStop(motorUpdateTimer, 0);
}

bool handleMotorUpdate() {
    EventBits_t bitsToCheck = MOTOR_UPDATE_BIT;
    EventBits_t bits = xEventGroupWaitBits(eventGroupHandle, bitsToCheck, false, false, 0);
    if((bits & MOTOR_UPDATE_BIT) != 0) {
        xEventGroupClearBits(eventGroupHandle, MOTOR_UPDATE_BIT);
        motorUpdate();
        return true;
    }

    return false;
}
