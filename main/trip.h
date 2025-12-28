#pragma once

#include <stdint.h>

// Reset Trip-1 (bijv. op long-press van mode-knop)
void resetTrip1(uint32_t distance);

// Trip-1 in 10m increments
uint32_t getTrip1(void);

// Trip-2 in 10m increments
uint32_t getTrip2(void);

// Total in 10m increments
uint32_t getTotal(void);

// Distance update vanuit de motor (afstand sinds motor power-on)
void distanceUpdate(uint32_t distance);

// Laad trip-data uit NVS (via batData)
void loadDistances(void);
