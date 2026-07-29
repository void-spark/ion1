#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CAL_SIZE 10
#define CAL_PAYLOAD_SIZE (CAL_SIZE + 1)
#define CAL_NVS_KEY_CALIB "calibration"

uint8_t *calibrationLoad();
bool calibrationSave(uint8_t *source);