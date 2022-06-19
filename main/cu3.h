#pragma once

#include <sys/unistd.h>

void showStateCu3(uint8_t level, bool lightOn, uint16_t speed, uint32_t trip);

void displayUpdateCu3(uint8_t assist, uint16_t speed, uint32_t trip1, uint32_t trip2);