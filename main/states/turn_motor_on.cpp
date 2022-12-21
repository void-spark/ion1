#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "blink.h"
#include "display.h"
#include "relays.h"
#include "bow.h"
#include "cmds.h"
#include "cu2.h"
#include "cu3.h"
#include "motor.h"
#include "states.h"

static const char *TAG = "turn_motor_on_state";

/**
 * Turn motor on state, motor relay was off, and now we want to turn everything on.
 * We:
 * - Turn on the display and start updating it
 * - Turn on the motor relay
 * - Send the 'motor on' command, and wait for a reply.
 * - Start updating the motor
 * - Do the 'auto pairing': set the display serial in the motor
 */
void toTurnMotorOnState(ion_state * state) {
    state->displayOn = true;

    // One long blink (0.5s)
    queueBlink(1, 500, 50);

    // Turn motor relay on
    setRelay(true);

    state->state = TURN_MOTOR_ON;
    state->step = 0;
}

void handleTurnMotorOnState(ion_state * state) {

    static uint8_t displaySerial[8] = {};
    static uint8_t motorSlot2Serial[8] = {};

#if CONFIG_ION_CU3
    const uint8_t nextStep = 1;
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
        displayUpdateCu2(false, ASS_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_SOLID, BLNK_SOLID, true, 25, 0xccc, 0xccccc);
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
        displayUpdateCu2(true, ASS_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, false, 10, 0xccc, 0xccccc);
    } else 
#else
    const uint8_t nextStep = 0;
#endif
    startDisplayUpdates();
    if(state->step == nextStep) {
        messageType message = {};
        // Motor on
        // Original BMS seems to repeat handoff till the motor responds, with 41ms between commands, but this should also work.
        // The exchange(..) method keeps trying until we get a (any) valid response message adressed to us
        exchange(cmdReq(MSG_MOTOR, MSG_BMS, CMD_MOTOR_ON), &message, 41 / portTICK_PERIOD_MS);
        state->doHandoffs = true;
    } else if(state->step == nextStep + 1) {
        motorUpdate();
        startMotorUpdates();
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
            // Serial already matched, no need to change it.
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
