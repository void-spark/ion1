#include "sdkconfig.h"
#include "storage.h"
#include "charge.h"

static struct batData *bat;

static uint32_t chargeFullMah = (CONFIG_ION_BAT_CHARGE * 1800); // increase * 1800 to conform to the relative current measurement.

bool full = true;

uint8_t getChargePercentage() {

    uint8_t percentageUsed = (uint8_t)(((float)bat->mah / (float)chargeFullMah) * 100.0f);
    if (percentageUsed > 100) percentageUsed = 100;

    uint8_t percentage = 100 - percentageUsed;

    if (percentage != bat->percentage) {
        bat->percentage = percentage;
        batDataSave();
    }

    return bat->percentage;
}

uint32_t getMv() {
    return bat->mv;
}

uint32_t getMah() {
    return bat->mah;
}

void chargeUpdate(uint32_t mv, uint32_t ma) {
    bat->mv = mv;
    bat->mah += ma;
}

void loadCharge() {
    bat = batDataGet();

    if (!batDataLoad()) {
        // Defaults als er nog geen data in NVS staat
        bat->percentage = 100;
        bat->mv = 0;
        bat->mah = 0;
    }
}

void resetCharge() {
    bat->percentage = 100;
    bat->mv = 0;
    bat->mah = 0;
    batDataSave();
}
