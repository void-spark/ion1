#pragma once

#include <stdint.h>

#define TRIP_NVS_KEY_TRIPDATA "tripdata"

struct tripData {
    uint32_t trip1;
    uint32_t trip2;
    uint32_t total;
};

// To reset on long press mode button
void resetTrip1(uint32_t distance);

// Trip-1 in 10m increments
uint32_t getTrip1();

// Trip-2 in 10m increments
uint32_t getTrip2();

// Total in 10m increments
uint32_t getTotal();

// Distance update from the motor, distance since motor power on in 10m increments
void distanceUpdate(uint32_t distance);

// Load distances from flash
void loadDistances();

// Write distances to flash
void saveDistances();
