#include <sys/unistd.h>

#include "cu2.h"
#include "bow.h"

static uint32_t digits(uint32_t value, size_t digits, size_t atleast) {
    uint32_t result = 0;
    uint32_t divider = 1;
    for(int pos = 0; pos < digits; pos++) {
        uint32_t digit = ((pos + 1 > atleast) && value < divider) ? 0xc : ((value / divider) % 10);
        result |= digit << (4 * pos);
        divider *= 10;
    }
    return result;
}

void showState(uint8_t level, bool lightOn, uint16_t speed, uint32_t trip) {
    uint16_t numTop = digits(speed, 3, 2);
    uint32_t numBottom = digits(trip / 100, 5, 1);
    displayUpdate(false, (assist_level)level, BLNK_SOLID, BLNK_OFF, BLNK_OFF, BLNK_SOLID, lightOn ? BLNK_SOLID : BLNK_OFF, BLNK_SOLID, BLNK_OFF, BLNK_SOLID, BLNK_SOLID, BLNK_SOLID, false, 50, numTop, numBottom);
}

void displayUpdate(bool setDefault,
                          assist_level assistLevel,
                          blink_speed assistBlink,
                          blink_speed wrench,
                          blink_speed total,
                          blink_speed trip,
                          blink_speed light,
                          blink_speed bars,
                          blink_speed comma,
                          blink_speed km,
                          blink_speed top,
                          blink_speed bottom,
                          bool miles,
                          uint8_t battery,
                          uint16_t topVal,
                          uint32_t bottomVal) {
    uint8_t assist = assistBlink;
    assist <<= assistLevel * 2;

    uint8_t segments1 = (wrench << 0) | (total << 2) | (trip << 4) | (light << 6);
    uint8_t segments2 = (bars << 0) | (comma << 4) | (km << 6);

    // Each digit is a nibble in the value:
    // Numbers are 0-9, a='-', b='b', c=' ', d='d', e='e', f='f'
    uint8_t numTop1 = ((uint8_t)(topVal >> 8) & 0x0f) | (miles ? 0xe0 : 0x00);
    uint8_t numTop2 = (uint8_t)(topVal >> 0);

    uint8_t numBottom1 = ((uint8_t)(bottomVal >> 16) & 0x0f) | (bottom << 4) | (top << 6);
    uint8_t numBottom2 = (uint8_t)(bottomVal >> 8);
    uint8_t numBottom3 = (uint8_t)(bottomVal >> 0);

    uint8_t cmd[] = {0xc1, 0x29, (uint8_t)(setDefault ? 0x27 : 0x26), assist, segments1, segments2, battery, numTop1, numTop2, numBottom1, numBottom2, numBottom3};

    exchange(cmd, sizeof(cmd));
}
