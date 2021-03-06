#pragma once

#include <sys/unistd.h>

enum display_type { DSP_SCREEN = 0, DSP_BAT_CHARGE, DSP_BAT };

void showStateCu3(uint8_t level, bool screenOn, bool lightOn, uint16_t speed, uint32_t trip);

void displayUpdateCu3(display_type type, bool screen, bool light, bool battery2, uint8_t assist, uint16_t speed, uint32_t trip1, uint32_t trip2);