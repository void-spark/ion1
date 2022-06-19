#include <sys/unistd.h>
#include "bow.h"

#include "cu3.h"

void showStateCu3(uint8_t level, bool lightOn, uint16_t speed, uint32_t trip) {
    displayUpdateCu3(level, speed, trip, trip);
}

/**
 * Pushes a display update to CU3
 * 
 * @param assist Assist mode: 0:'0', 1:'1', 2:'2', 3:'3', 4:'P', 5:'R', 7:'4'
 * @param speed Speed in 100m/h increments (or km/h * 10), display will round them to nearest 0.5 km/h
 * @param trip1 Trip 1 distance in 10m increments
 * @param trip2 Trip 2 distance in 10m increments
 */
void displayUpdateCu3(uint8_t assist, uint16_t speed, uint32_t trip1, uint32_t trip2) {
    uint8_t byte0 = 0x03; // 0 screen off ?? 1 shows battery + charging, 2 shows battery, 3 screen, 
                            // Other: Charge state, >0 = busy display
    uint8_t byte2 = 0x08; // 00(0000) Light off(??), 08(1000) light on, 09(1001) light auto. But also (auto??) turns off display at 00 ?!???
                            // Other:   0x0A(1010) Backlight off, 0x0B(1011) = Backlight on, 0x0C(1100) = Range extender?

    uint8_t cmd[] = {0xc1, 0x2d, 0x28, 
    byte0, 
    assist, 
    byte2, 
    (uint8_t)(speed >> 8), (uint8_t)(speed >> 0),
    (uint8_t)(trip1 >> 24), (uint8_t)(trip1 >> 16), (uint8_t)(trip1 >> 8), (uint8_t)(trip1 >> 0),
    (uint8_t)(trip2 >> 24), (uint8_t)(trip2 >> 16), (uint8_t)(trip2 >> 8), (uint8_t)(trip2 >> 0)};
    messageType message = {};
    readResult result = exchange(cmd, sizeof(cmd), &message, 225 / portTICK_PERIOD_MS );
}
