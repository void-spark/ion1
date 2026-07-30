#include "cmds.h"
#include "storage.h"
#include "calibration.h"
#include <string.h>

#define CAL_SIZE 10
#define CAL_NVS_KEY_CALIB "calibration"

bool handleCalibrationMessage(const messageType& message) {
    if(message.type == MSG_CMD_REQ && message.payloadSize == 4 && message.command == CMD_GET_DATA && message.payload[1] == 0x38 && message.payload[3] == 0x3a) {
        // GET DATA 38 and 3a

        // Fallback calibration (10 bytes)
        // This is from a test on an old sparta. It is probably not very good for most,
        // but it's a better starting point than just 0s.
        // I wonder what happens if we use status code 1 instead? Will the motor auto calibrate?
        // Structure here:
        // 0x94: Value is signed integer, size 2, more values follow.
        // 0x38: ID
        // 0x4b15: the actual first value: 19221
        // 0x28: Value is float, size 4, no values follow.
        // 0x3a: ID
        // 0x3e917950: Second value: 0.284128666
        static const uint8_t fallback[CAL_SIZE] = { 0x94, 0x38, 0x4b, 0x15, 0x28, 0x3a, 0x3e, 0x91, 0x79, 0x50 };

        static uint8_t payload[CAL_SIZE + 1] = {};

        // First byte is status byte. 0x00 is Ok, 0x01 is not found.
        payload[0] = 0x00;

        // Try reading calibration from NVS.
        if(!dataLoad(CAL_NVS_KEY_CALIB, payload + 1, CAL_SIZE)) {
            // Failed to load calibration, use fallback instead.
            memcpy(payload + 1, fallback, CAL_SIZE);
        }

        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return true;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == CAL_SIZE && message.command == CMD_PUT_DATA && message.payload[1] == 0x38 && message.payload[5] == 0x3a) {
        // PUT DATA 38 and 3a

        if(!dataSave(CAL_NVS_KEY_CALIB, message.payload, CAL_SIZE)) {
            return true;
        }

        uint8_t payload[] = {0x00};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return true;
    }

    return false;
}