#pragma once

#include "sdkconfig.h"
#if CONFIG_ION_CU3

#include "bow.h"
#include <sys/unistd.h>

enum display_type { DSP_SCREEN = 0, DSP_BAT_CHARGE, DSP_BAT };

void displayUpdateCu3(display_type type, bool screen, bool light, bool battery2, uint8_t assist, uint16_t speed, uint32_t trip1, uint32_t trip2);

bool handleCu3Message(const messageType& message);

#endif
