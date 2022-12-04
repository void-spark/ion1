
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "blink.h"
#include "trip.h"
#include "states.h"

void toMotorOffState(ion_state * state) {

    queueBlink(4, 100, 300);

    saveDistances();

    state->state = MOTOR_OFF;
    state->step = 0;
}

void handleMotorOffState(ion_state * state, bool modeShortPress, bool wakeup) {
    // Motor is off, but we may still get handoff messages from it, or the display (CU3).
    if(modeShortPress || wakeup) {
        toTurnMotorOnState(state);
    }
}
