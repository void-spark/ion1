#pragma once

#include <stdint.h>

uint8_t getChargePercentage(void);

uint32_t getMv(void);

uint32_t getMah(void);

void chargeUpdate(uint32_t mv, uint32_t mah);

void loadCharge(void);

void resetCharge(void);
