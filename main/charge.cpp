#include "sdkconfig.h"
#include "storage.h"
#include "charge.h"

#define CHARGE_FILE "/littlefs/charge.bin"
// note that the Ah is not real Ah but a relative Ah measurement based on ADC and timer.

struct chargeStruct {

    uint32_t mv;

    uint32_t mah;
};

static chargeStruct charge;

static uint32_t chargeFullMah = (CONFIG_ION_BAT_CHARGE * 2400); // increase * 2400 to conform to the relative current measurement.

bool full = true;


uint8_t getChargePercentage() {
	uint8_t percentageUsed = (uint8_t) (((float)charge.mah / (float)chargeFullMah) * 100.0f);
	if(percentageUsed > 100) percentageUsed = 100;
	return 100 - percentageUsed; //percentage left
}

uint32_t getMv() {
    return charge.mv;
}

uint32_t getMah() {
    return charge.mah;
}

void chargeUpdate(uint32_t mv, uint32_t ma) {
	/*
	uint32_t diff = (charge.mv > mv) ? (charge.mv - mv) : (mv - charge.mv);
	if (full) {
		charge.mah = chargeFullMah;
	}
	*/
	charge.mv = mv;
    charge.mah += ma;
	/*
	if(charge.mah > chargeFullMah) {
		charge.mah = 0;
	}
	*/
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
	charge.mv = 0;
	charge.mah = 0;
}
