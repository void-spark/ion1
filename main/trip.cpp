#include "storage.h"
#include "trip.h"

#define DISTANCE_FILE "/spiffs/distance.bin"

struct distancesStruct {
    // Trip-1 in 10m increments
    uint32_t trip1;

    // Trip-2 in 10m increments
    uint32_t trip2;

    // Total in 10m increments
    uint32_t total;
};

static distancesStruct distances;

static uint32_t lastDistance = 0;

uint32_t getTrip1() {
    return distances.trip1;
}

uint32_t getTrip2() {
    return distances.trip2;
}

uint32_t getTotal() {
    return distances.total;
}

void distanceUpdate(uint32_t distance) {
    if(distance < lastDistance) {
        // We expect this only happens when the motor reset (powered off and on).
        // Which means the motor started at 0 again.
        // We could reset when we know we power off the motor instead, but what if we don't have a relay (or it's broken)?
        lastDistance = 0;
    }

    uint32_t delta = distance - lastDistance;

    distances.trip1 += delta;
    distances.trip2 += delta;
    distances.total += delta;

    lastDistance = distance;
}

void loadDistances() {
    if(fileExists(DISTANCE_FILE)) {
        readData(DISTANCE_FILE, &distances, sizeof(distances));
    }
}

void saveDistances() {
    writeData(DISTANCE_FILE, &distances, sizeof(distances));
}
