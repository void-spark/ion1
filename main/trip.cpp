#include "trip.h"

// Trip-1 in 10m increments
static uint32_t trip1;

// Trip-2 in 10m increments
static uint32_t trip2;

uint32_t getTrip1() {
    return trip1;
}

uint32_t getTrip2() {
    return trip2;
}

void distanceUpdate(uint32_t distance) {
    trip1 = distance;
    trip2 = distance;
}
