#include "storage.h"
#include "trip.h"

static struct batData *bat;

static uint32_t lastDistance = 0;

void resetTrip1(uint32_t distance) {
    bat->trip1 = distance;
}

uint32_t getTrip1() {
    return bat->trip1;
}

uint32_t getTrip2() {
    return bat->trip2;
}

uint32_t getTotal() {
    return bat->total;
}

void distanceUpdate(uint32_t distance) {

    if (distance < lastDistance) {
        // Motor is opnieuw opgestart → teller terug naar 0
        lastDistance = 0;
    }

    uint32_t delta = distance - lastDistance;

    bat->trip1 += delta;
    bat->trip2 += delta;
    bat->total += delta;

    lastDistance = distance;
}

void loadDistances() {
    bat = batDataGet();

    if (!batDataLoad()) {
        // Defaults als er nog geen data in NVS staat
        bat->trip1 = 0;
        bat->trip2 = 0;
        bat->total = 0;
    }
}
