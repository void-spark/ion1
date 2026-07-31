#pragma once

// Initialisatie en teardown
void adc_init();
void adc_teardown();

// Voltage measurement
void measureBat();
uint32_t getBatMv();
uint8_t getBatPercentage();

// Current Measurement
void measureCurrent();
uint32_t getBatMa();
uint32_t getBatMah();