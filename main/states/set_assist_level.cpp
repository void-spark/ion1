#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "blink.h"
#include "cmds.h"
#include "bow.h"
#include "display.h"
#include "states.h"

/**
 * This state changes the assist level, and turns on assist if it is required.
*/
void toSetAssistLevelState(ion_state * state) {
    if(state->level == 0) {
        queueBlink(2, 250, 50);
    } else {
        queueBlink(state->level, 100, 50);
    }

    state->state = SET_ASSIST_LEVEL;
    state->step = 0;
}

void handleSetAssistLevelState(ion_state * state) {
    if(state->level == 0) {
        if(state->assistOn) {
            exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_ASSIST_OFF));
            state->assistOn = false;

            // TODO: Start waiting for MYSTERY BAT COMMAND 12 (with arg 0), while
            // doing handoffs. So this should be a state? And in the handoff we
            // signal the next state.

            state->levelSet = state->level;
            requestDisplayUpdate();
        }
        toMotorOnState(state);
        return;
    } else {
        if(!state->assistOn) {
            exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_ASSIST_ON));
            state->assistOn = true;

            // TODO: Start waiting for MYSTERY BAT COMMAND 12 (with arg 1), while
            // doing handoffs. So this should be a state? And in the handoff we
            // signal the next state.
        } else {
            uint8_t payload[] = {state->level};
            exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_SET_ASSIST_LEVEL, payload, sizeof(payload)));

            state->levelSet = state->level;
            requestDisplayUpdate();

            toMotorOnState(state);
            return;
        }
    }     
}
