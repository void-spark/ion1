#include "storage.h"
#include "trip.h"

static struct tripData data;

static uint32_t lastDistance = 0;

void resetTrip1(uint32_t distance) {
    data.trip1 = distance;
}

uint32_t getTrip1() {
    return data.trip1;
}

uint32_t getTrip2() {
    return data.trip2;
}

uint32_t getTotal() {
    return data.total;
}

void distanceUpdate(uint32_t distance) {
    if(distance < lastDistance) {
        // We expect this only happens when the motor reset (powered off and on).
        // Which means the motor started at 0 again.
        // We could reset when we know we power off the motor instead, but what if we don't have a relay (or it's broken)?
        lastDistance = 0;
    }

    uint32_t delta = distance - lastDistance;

    data.trip1 += delta;
    data.trip2 += delta;
    data.total += delta;

    lastDistance = distance;
}

void loadDistances() {
    dataLoad(TRIP_NVS_KEY_TRIPDATA, &data, sizeof(data));
}

void saveDistances() {
    dataSave(TRIP_NVS_KEY_TRIPDATA, &data, sizeof(data));
}
