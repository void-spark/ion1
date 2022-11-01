#include <sys/unistd.h>
#include "bow.h"
#include "cmds.h"

#include "cu3.h"

void showStateCu3(uint8_t level, bool screenOn, bool lightOn, uint16_t speed, uint32_t trip1, uint32_t trip2) {
    displayUpdateCu3(DSP_SCREEN, screenOn, lightOn, false, level, speed, trip1, trip2);
}

/**
 * Pushes a display update to CU3
 * 
 * @param type The display type (Normal screen, battery + % + charging, battery + %)
 * @param screen Screen on or off(=logo)
 * @param light Light on or off
 * @param battery2 Show the range extender icon + charge.
 * @param assist Assist mode: 0:'0', 1:'1', 2:'2', 3:'3', 4:'P', 5:'R', 7:'4'
 * @param speed Speed in 100m/h increments (or km/h * 10), display will round them to nearest 0.5 km/h
 * @param trip1 Trip 1 distance in 10m increments
 * @param trip2 Trip 2 distance in 10m increments
 */
void displayUpdateCu3(display_type type, bool screen, bool light, bool battery2, uint8_t assist, uint16_t speed, uint32_t trip1, uint32_t trip2) {
    uint8_t byte0 = type;
    if(type == DSP_SCREEN && assist > 0) {
        // Not sure why, but the original seems to do this.
        byte0 = 0x03;
    }
    uint8_t byte2 = (light ? 0x01 : 0x00) | (battery2 ? 0x04 : 0x00) | (screen ? 0x08 : 0x00);

    uint8_t payload[] = {byte0, assist, byte2, 
    (uint8_t)(speed >> 8), (uint8_t)(speed >> 0),
    (uint8_t)(trip1 >> 24), (uint8_t)(trip1 >> 16), (uint8_t)(trip1 >> 8), (uint8_t)(trip1 >> 0),
    (uint8_t)(trip2 >> 24), (uint8_t)(trip2 >> 16), (uint8_t)(trip2 >> 8), (uint8_t)(trip2 >> 0)};
    messageType message = {};
    readResult result = exchange(cmdReq(MSG_DISPLAY, MSG_BMS, 0x28, payload, sizeof(payload)), &message, 225 / portTICK_PERIOD_MS );
}

/**
 * The max value for batVal (100%)
 */
uint16_t cu3BatMaxValue() {
    return 11000;
}

/** 
 * Calculate the Bat. value for CU3, range is something like -10% - 100%
*/
uint16_t toCu3BatValue(uint8_t batPercentage) {

    uint16_t batMax = cu3BatMaxValue(); 

    // CU3 seems to calculate % with something close to:
    // floor( (val - (0.091 * max)) / 0.009 * max )
    // Below we do the reverse.
    uint32_t offsetK = (91 * batMax);
    uint32_t onePercentK = (9 * batMax);
    uint32_t valueK = offsetK + onePercentK * batPercentage + onePercentK / 2;
    return (uint16_t) (valueK / 1000);
}
