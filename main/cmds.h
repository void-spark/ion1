#pragma once

#include "bow.h"

messageType handoffMsg(uint8_t target);
messageType pingReq(uint8_t target, uint8_t source);
messageType pingResp(uint8_t target, uint8_t source);
messageType cmdReq(uint8_t target, uint8_t source, uint8_t command);
messageType cmdResp(uint8_t target, uint8_t source, uint8_t command);
messageType cmdReq(uint8_t target, uint8_t source, uint8_t command, uint8_t *payload, size_t payloadSize);
messageType cmdResp(uint8_t target, uint8_t source, uint8_t command, uint8_t *payload, size_t payloadSize);
