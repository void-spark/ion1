#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

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
bool calibrationLoad(void *out_value, size_t length);
bool calibrationSave(const void *value, size_t length);
