#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "blink.h"
#include "cmds.h"
#include "bow.h"
#include "display.h"
#include "relays.h"
#include "motor.h"
#include "states.h"

void toTurnMotorOffState(ion_state * state) {
    state->displayOn = false;
    queueBlink(2, 400, 50);

    state->state = TURN_MOTOR_OFF;
    state->step = 0;
}

void handleTurnMotorOffState(ion_state * state) {
    if(state->assistOn) {
        exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_ASSIST_OFF));
        state->assistOn = false;

        // TODO: Start waiting for MYSTERY BAT COMMAND 12 (with arg 0), while
        // doing handoffs. So this should be a state? And in the handoff we
        // signal the next state.
    } else {
        if(state->step == 0) {
            stopMotorUpdates();
            stopDisplayUpdates();

            state->motorOffAck = false;
            uint8_t payload[] = {0x00};
            exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_MOTOR_OFF, payload, sizeof(payload)));
            // NOTE: XHP after this will stop responding to handoff messages after some time (and a put data message)

            state->step++;
        } else if (state->step == 1)
            if(state->motorOffAck) {
                setRelay(false);

                toMotorOffState(state);
            }
    }
}
