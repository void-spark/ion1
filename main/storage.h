#pragma once

#include <sys/unistd.h>

void init_spiffs();
bool calibrationFileExists();

// Reads calibration data to the given buffer, data is 10 bytes long
bool readCalibrationData(uint8_t * target);

// Writes calibration data from the given buffer, data is 10 bytes long
bool writeCalibrationData(uint8_t * source);
