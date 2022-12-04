#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "blink.h"
#include "cmds.h"
#include "bow.h"
#include "states.h"

void toCalibrateState(ion_state * state) {
    queueBlink(10, 100, 100);

    state->state = START_CALIBRATE;
    state->step = 0;
}

void handleCalibrateState(ion_state * state) {
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
