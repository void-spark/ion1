#include "sdkconfig.h"
#include "storage.h"
#include "charge.h"

// note that Ah is not real Ah but a relative Ah measurement based on ADC and timer.

struct chargeStruct {

    uint8_t percentage;

    uint32_t mv;

    uint32_t mah;
};

static chargeStruct charge;

static uint32_t chargeFullMah = (CONFIG_ION_BAT_CHARGE * 1800); // increase * 1800 to conform to the relative current measurement.

bool full = true;

uint8_t getChargePercentage() {

	uint8_t percentageUsed = (uint8_t) (((float)charge.mah / (float)chargeFullMah) * 100.0f);
	if(percentageUsed > 100) percentageUsed = 100;
    uint8_t percentage = 100 - percentageUsed; //percentage left;
	
	if(percentage != charge.percentage){
		charge.percentage = percentage;
		saveCharge();
	}
	
	return charge.percentage;
}

uint32_t getMv() {
    return charge.mv;
}

uint32_t getMah() {
    return charge.mah;
}

void chargeUpdate(uint32_t mv, uint32_t ma) {
	charge.mv = mv;
    charge.mah += ma;
}

void loadCharge() {
    if(fileExists(CHARGE_FILE)) {
        readData(CHARGE_FILE, &charge, sizeof(charge));
    }
}

void saveCharge() {
    writeData(CHARGE_FILE, &charge, sizeof(charge));
}

void resetCharge() {
    charge.percentage = 100;
	charge.mv = 0;
	charge.mah = 0;
}
