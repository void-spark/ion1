#pragma once

void adc_init();
void measureBat();
uint32_t getBatMv();
uint8_t getBatPercentage();
void adc_teardown();
