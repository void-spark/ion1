#include "trip.h"

// Trip-1 in 10m increments
static uint32_t trip1;

// Trip-2 in 10m increments
static uint32_t trip2;

// Total in 10m increments
static uint32_t total;

static uint32_t lastDistance = 0;

uint32_t getTrip1() {
    return trip1;
}

uint32_t getTrip2() {
    return trip2;
}

uint32_t getTotal() {
    return total;
}

void distanceUpdate(uint32_t distance) {
    if(distance < lastDistance) {
        // We expect this only happens when the motor reset (powered off and on).
        // Which means the motor started at 0 again.
        // We could reset when we know we power off the motor instead, but what if we don't have a relay (or it's broken)?
        lastDistance = 0;
    }

    uint32_t delta = distance - lastDistance;

    trip1 += delta;
    trip2 += delta;
    total += delta;

    lastDistance = distance;
}
