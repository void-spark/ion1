#pragma once

#include <sys/unistd.h>
#include "freertos/FreeRTOS.h"

#define MSG_HANDOFF 0x0
#define MSG_CMD_REQ 0x1
#define MSG_CMD_RESP 0x2
#define MSG_PING_REQ 0x4
#define MSG_PING_RESP 0x3

#define MSG_MOTOR 0x0
#define MSG_BMS 0x2
#define MSG_DISPLAY 0xC

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
void writeMessage(const messageType& message);
readResult exchange(const messageType& outMessage, messageType *inMessage, const TickType_t timeout);
readResult exchange(const messageType& outMessage, messageType *inMessage);
void exchange(const messageType& outMessage);
