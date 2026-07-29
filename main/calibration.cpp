#include "storage.h"
#include "calibration.h"
#include <string.h>

uint8_t *calibrationLoad() {
    static uint8_t payload[CAL_PAYLOAD_SIZE];

    // First byte is status byte. 0x00 is Ok, 0x01 is not found.
    payload[0] = 0x00;

    // Fallback calibration (10 bytes)
    // This is from a test on an old sparta. It is probably not very good for most,
    // but it's a better starting point than just 0s.
    // I wonder what happens if we use status code 1 instead? Will the motor auto calibrate?
    static const uint8_t fallback[CAL_SIZE] = {
        0x94, 0x38, 0x4b, 0x15, 0x28, 0x3a, 0x3e, 0x91, 0x79, 0x50
    };

    // Try reading calibration from NVS.
    if(dataLoad(CAL_NVS_KEY_CALIB, payload + 1, CAL_SIZE)) {
        // Calibration loaded successfully, return it.
        return payload;
    }

    // Failed to load calibration, return fallback instead.
    memcpy(payload + 1, fallback, CAL_SIZE);
    return payload;
}

bool calibrationSave(uint8_t *source) {
    return dataSave(CAL_NVS_KEY_CALIB, source, CAL_SIZE);
}
