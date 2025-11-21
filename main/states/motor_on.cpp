#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "relays.h"
#include "states.h"

static const char *TAG = "motor_on_state";

/**
 * Motor on state, we handle these user actions:
 * - Calibrate request, we go to calibrate state.
 * - Mode press on CU2, we change the wanted assist level.
 * 
 * If the wanted assist level does not match the actual, we go to set assist level state.
 * 
 * We also keep of how long ago we moved, if we are not in a assist state.
 * After 10 seconds of not moving we go to the turn motor off state.
*/
void toMotorOnState(ion_state * state) {
    state->state = MOTOR_ON;
    state->step = 0;
}

void handleMotorOnState(ion_state * state, bool modeShortPress, bool lightLongPress, bool calibrate) {

    static int64_t lastMoving = 0;
    int64_t now = esp_timer_get_time();

    if(state->step == 0 || state->speed > 0 || state->levelSet != 0) {
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
    if((state->level == 0x00 && getLight() == false && lightLongPress) || calibrate) {
        toCalibrateState(state);
        return;
    } 

    // Handle level change request from CU2 display, by pressing mode button.
    if(modeShortPress) {
        state->level = (state->level + 1) % 4;
    }

    if(state->level != state->levelSet) {
        toSetAssistLevelState(state);
        return;
    }            
}
