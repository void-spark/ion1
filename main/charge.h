#pragma once

#include <sys/unistd.h>

uint8_t getChargePercentage();

uint32_t getMv();

uint32_t getMah();

void chargeUpdate(uint32_t mv, uint32_t mah);

void loadCharge();

void saveCharge();

void resetCharge();
