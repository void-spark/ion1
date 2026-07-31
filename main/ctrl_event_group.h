#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const int BUTTON_MODE_SHORT_PRESS_BIT = BIT0;
static const int BUTTON_MODE_LONG_PRESS_BIT = BIT1;
static const int BUTTON_LIGHT_SHORT_PRESS_BIT = BIT2;
static const int BUTTON_LIGHT_LONG_PRESS_BIT = BIT3;
static const int IGNORE_HELD_BIT = BIT4;
static const int WAKEUP_BIT = BIT5;
static const int CALIBRATE_BIT = BIT6;
static const int MEASURE_BAT_BIT = BIT7;

void initControlEventGroup();

void setControlBits(const EventBits_t uxBitsToSet);

void clearControlBits(const EventBits_t uxBitsToSet);

EventBits_t waitControlBits(const EventBits_t uxBitsToWaitFor,
                            const BaseType_t xClearOnExit,
                            const BaseType_t xWaitForAllBits,
                            TickType_t xTicksToWait);