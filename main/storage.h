#pragma once
#include <stdint.h>
#include <stdbool.h>

struct batData {
    uint32_t trip1;
    uint32_t trip2;
    uint32_t total;

    uint8_t percentage;
    uint32_t mv;
    uint32_t mah;
};

// Init NVS + defaults
void storageInit(void);

// batData API
struct batData *batDataGet(void);
bool batDataLoad(void);
bool batDataSave(void);

// calibration API
uint8_t *calibrationLoad(void);
bool calibrationSave(uint8_t *source);
