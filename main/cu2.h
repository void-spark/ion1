#pragma once

#include <sys/unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

enum assist_level { ASS_OFF = 0, ASS_ECO, ASS_NORMAL, ASS_POWER };
enum blink_speed { BLNK_OFF = 0, BLNK_FAST, BLNK_SLOW, BLNK_SOLID };

void initCu2(EventGroupHandle_t eventGroupHandle,
             const int buttonModeShortPressBit,
             const int buttonModeLongPressBit,
             const int buttonLightShortPressBit,
             const int buttonLightLongPressBit,
             const int ignoreHeldBit);

void buttonCheck();

void startButtonCheck();

void stopButtonCheck();

void ignorePress();

bool cu2HandleDisplayUpdate();

void displayUpdateCu2(bool setDefault,
                   assist_level assistLevel,
                   blink_speed assistBlink,
                   blink_speed wrench,
                   blink_speed total,
                   blink_speed trip,
                   blink_speed light,
                   blink_speed bars,
                   blink_speed comma,
                   blink_speed km,
                   blink_speed top,
                   blink_speed bottom,
                   bool miles,
                   uint8_t batPercentage,
                   uint16_t topVal,
                   uint32_t bottomVal);
