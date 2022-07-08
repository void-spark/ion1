#pragma once

#include <sys/unistd.h>

void queueBlink(size_t blinks, uint32_t onTime, uint32_t offTime);
void initBlink();
