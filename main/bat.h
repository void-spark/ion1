#pragma once

void adc_init();
uint32_t measureBatMv();
uint8_t batMvToPercentage(uint32_t batVoltageMv);
void adc_teardown();
