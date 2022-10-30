#pragma once

#include <sys/unistd.h>

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
