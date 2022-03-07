#pragma once

#include <sys/unistd.h>
#include "freertos/FreeRTOS.h"

struct messageType {
    // Payload length is indicated by one nibble, so max value 0xF (15).
    // Payload length excludes the starting byte, 2 header bytes, command byte, and crc byte.
    // So total length = payload + 5, and max length is 15 + 5 = 20.
    uint8_t data[20];
    uint8_t length;

    // Header values, we use -1 as 'unset'.
    uint8_t target;
    uint8_t source;
    int8_t type;
    int8_t size;

    bool wakeup;
    bool timeout;
};

void initUart();
messageType readMessage(TickType_t timeout);
messageType readMessage();
void writeMessage(uint8_t *message, uint8_t messageLen);
messageType exchange(uint8_t *cmd, size_t cmdLen, const TickType_t timeout);
messageType exchange(uint8_t *cmd, size_t cmdLen);
