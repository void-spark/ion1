#include "esp_log.h"
#include "bow.h"
#include "cmds.h"
#include "display.h"
#include "blink.h"
#include "relays.h"
#include "motor.h"
#include "states.h"

static const char *TAG = "charging_state";

void toChargingState(ion_state * state) {
    state->state = CHARGING;
    state->step = 0;

    // We do want to show charge state
    state->displayOn = true;

    // Enable the motor relay for boards with the voltage divider after the relay.
    setRelay(true);

    // No need for these while charging.
    stopMotorUpdates();

    queueBlink(5, 500, 500);

    // Show charging on the display
    requestDisplayUpdate();
    startDisplayUpdates();
}

void handleChargingState(ion_state * state, bool chargePin) {

    // First set assist level to 0, if it's set higher
    if(state->assistOn && state->levelSet > 0) {
        uint8_t payload[] = {0};
        exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_SET_ASSIST_LEVEL, payload, sizeof(payload)));
        state->level = 0;
        state->levelSet = 0;
    }

    // Then set assist off, if it's on
    if(state->assistOn) {
        exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_ASSIST_OFF));
        state->assistOn = false;
    }


    if(chargePin) {
        // Wait till someone unplugs the charger.
        return;
    }

    setRelay(false);
    // Go through the steps to fully turn motor on, seems a decent state to be in after charging, it will go back to off/idle if we don't move.
    toTurnMotorOnState(state);
    return;
}
