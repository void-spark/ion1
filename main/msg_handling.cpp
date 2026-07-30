#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "bytes.h"
#include "bow.h"
#include "cmds.h"
#include "ctrl_event_group.h"
#include "cu3.h"
#include "calibration.h"
#include "trip.h"
#include "display.h"
#include "relays.h"
#include "msg_handling.h"

static const char *TAG = "msg_handling";

messageHandlingResult handleMessage(ion_state * state) {
    messageType message = {};
    readResult result;
    do {
        // Replies to handoff should be a lot quicker then 250ms
        result = readMessage(&message, 250 / portTICK_PERIOD_MS);
        if(result == MSG_TIMEOUT) {
            // Most likely the motor turned off, after we told it to (XHP, Toprun doesn't seem to stop)
            // Or if we're handing of to CU3, it might not be there.
            return HANDOFF_TIMEOUT;
        }
    } while(result != MSG_OK || message.target != MSG_BMS);

    if(message.type == MSG_HANDOFF) {
        // Handoff back to us
        return CONTROL_TO_US;
    } else if(message.type == MSG_PING_REQ) {
        // ESP_LOGI(TAG, "|PING");
        writeMessage(pingResp(message.source, MSG_BMS));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 0 && message.command == 0x01) {
        // MYSTERY BATTERY COMMAND 01
        uint8_t payload[] = {0x02, 0x02};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 0 && message.command == CMD_BAT_STATUS_MOTOR_OFF) {
        state->motorOffAck = true;
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 1 && message.command == CMD_BAT_STATUS_ASSIST) {
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 0 && message.command == CMD_BAT_WAKEUP) {
        setControlBits(WAKEUP_BIT);
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 1 && message.command == CMD_BAT_CALIBRATE) {
        setControlBits(CALIBRATE_BIT);
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 1 && message.command == CMD_BAT_SET_LIGHT) {
        setLight(message.payload[0]);
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 1 && message.command == CMD_BAT_SET_ASSIST_LEVEL) {
        state->level = message.payload[0];
        writeMessage(cmdResp(message.source, MSG_BMS, message.command));
        return CONTROL_TO_SENDER;
#if CONFIG_ION_CU3
    } else if(handleCu3Message(message)) {
        return CONTROL_TO_SENDER;
#endif
    } else if(handleCalibrationMessage(message)) {
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 2 && message.command == CMD_GET_DATA && message.payload[1] == 0x2a) {
        // GET DATA 002a 00:2a(Unknown)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], 0x01};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return CONTROL_TO_SENDER;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 10 && message.command == CMD_PUT_DATA && message.payload[1] == 0xc0 && message.payload[5] == 0xc1) {
        // PUT DATA c0/c1
        state->speed = toUint16(message.payload, 2);
        distanceUpdate(toUint32(message.payload, 6));

        uint8_t payload[] = {0x00};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        requestDisplayUpdate();
        return CONTROL_TO_SENDER;
    }

    ESP_LOGI(TAG, "Unexpected: Tgt:%d, Src:%d, Type:%d, Command:%d", message.target, message.source, message.type, message.command);
    ESP_LOG_BUFFER_HEX(TAG, message.payload, message.payloadSize);

    return CONTROL_TO_SENDER;
}