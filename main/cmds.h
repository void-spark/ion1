#pragma once

#include "bow.h"

// Generic commands
#define CMD_GET_DATA 0x08
#define CMD_PUT_DATA 0x09

// Battery commands
#define CMD_BAT_STATUS_MOTOR_OFF 0x11
#define CMD_BAT_STATUS_ASSIST 0x12
#define CMD_BAT_WAKEUP 0x14
#define CMD_BAT_CALIBRATE 0x1b
#define CMD_BAT_SET_LIGHT 0x1c
#define CMD_BAT_SET_ASSIST_LEVEL 0x1d

// Display commands
#define CMD_GET_SERIAL 0x20
#define CMD_BUTTON_POLL 0x22

// Motor commands
#define CMD_MOTOR_ON 0x30
#define CMD_MOTOR_OFF 0x31
#define CMD_ASSIST_ON 0x32
#define CMD_ASSIST_OFF 0x33
#define CMD_SET_ASSIST_LEVEL 0x34
#define CMD_CALIBRATE 0x35

messageType handoffMsg(uint8_t target);
messageType pingReq(uint8_t target, uint8_t source);
messageType pingResp(uint8_t target, uint8_t source);
messageType cmdReq(uint8_t target, uint8_t source, uint8_t command);
messageType cmdResp(uint8_t target, uint8_t source, uint8_t command);
messageType cmdReq(uint8_t target, uint8_t source, uint8_t command, uint8_t *payload, size_t payloadSize);
messageType cmdResp(uint8_t target, uint8_t source, uint8_t command, uint8_t *payload, size_t payloadSize);
