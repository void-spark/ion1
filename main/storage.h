#pragma once

#include <sys/unistd.h>

#define CALIBRATION_FILE "/littlefs/calibration.bin"

#define DISTANCE_FILE "/littlefs/distance.bin"

#define CHARGE_FILE "/littlefs/charge.bin"

void init_fs();

// Is a calibration file stored?
bool calibrationFileExists();

// Reads calibration data to the given buffer, data is 10 bytes long
bool readCalibrationData(uint8_t * target);

// Writes calibration data from the given buffer, data is 10 bytes long
bool writeCalibrationData(uint8_t * source);

bool fileExists(const char * path);
bool readData(const char * path, void * target, size_t size);
bool writeData(const char * path, void * source, size_t size);
