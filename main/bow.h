#pragma once

#include <sys/unistd.h>
#include "freertos/FreeRTOS.h"

struct messageType {
    // The target of the message
    uint8_t target;
    // The source of the message, 0x00 if not used (check type)
    uint8_t source;
    // The type of the message
    uint8_t type;
    // The command byte of the message, 0x00 if not used (check type)
    uint8_t command;

    // The payload of the message, and it's length.
    // Payload length is indicated by one nibble, so max value 0xF (15).
    uint8_t payload[15];
    size_t payloadSize;
};

enum readResult {
    // Reading timed out before getting a full message.
    MSG_TIMEOUT,
    // We got a '0x00' byte instead of a message.
    MSG_WAKEUP,
    // A message was read, but the CRC is invalid.
    MSG_CRC_ERROR,
    // For internal use, no error but message is not yet complete.
    MSG_CONTINUE,
    // Message received.
    MSG_OK
};

void initUart();
readResult readMessage(messageType *message, TickType_t timeout);
readResult readMessage(messageType *message);
void writeMessage(uint8_t *message, uint8_t messageLen);
readResult exchange(uint8_t *cmd, size_t cmdLen, messageType *inMessage, const TickType_t timeout);
readResult exchange(uint8_t *cmd, size_t cmdLen, messageType *inMessage);
void exchange(uint8_t *cmd, size_t cmdLen);
