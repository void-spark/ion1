#include "sdkconfig.h"
#if CONFIG_ION_CU3

#include <sys/unistd.h>
#include "esp_timer.h"
#include "bytes.h"
#include "bow.h"
#include "cmds.h"
#include "bat.h"
#include "trip.h"

#include "cu3.h"

// Time offset in seconds, add this to time since boot to get real time.
static uint32_t offset = 0;

/**
 * Pushes a display update to CU3
 * 
 * @param type The display type (Normal screen, battery + % + charging, battery + %)
 * @param screen Screen on or off(=logo)
 * @param light Light on or off
 * @param battery2 Show the range extender icon + charge.
 * @param assist Assist mode: 0:'0', 1:'1', 2:'2', 3:'3', 4:'P', 5:'R', 7:'4'
 * @param speed Speed in 100m/h increments (or km/h * 10), display will round them to nearest 0.5 km/h
 * @param trip1 Trip 1 distance in 10m increments
 * @param trip2 Trip 2 distance in 10m increments
 */
void displayUpdateCu3(display_type type, bool screen, bool light, bool battery2, uint8_t assist, uint16_t speed, uint32_t trip1, uint32_t trip2) {
    uint8_t byte0 = type;
    if(type == DSP_SCREEN && assist > 0) {
        // Not sure why, but the original seems to do this.
        byte0 = 0x03;
    }
    uint8_t byte2 = (light ? 0x01 : 0x00) | (battery2 ? 0x04 : 0x00) | (screen ? 0x08 : 0x00);

    uint8_t payload[] = {byte0, assist, byte2, 
    FROM_UINT16(speed),
    FROM_UINT32(trip1),
    FROM_UINT32(trip2)};
    messageType message = {};
    readResult result = exchange(cmdReq(MSG_DISPLAY, MSG_BMS, 0x28, payload, sizeof(payload)), &message, 225 / portTICK_PERIOD_MS );
}

/**
 * The max value for batVal (100%)
 */
static uint16_t cu3BatMaxValue() {
    return 11000;
}

/** 
 * Calculate the Bat. value for CU3, range is something like -10% - 100%
*/
static uint16_t toCu3BatValue(uint8_t batPercentage) {

    uint16_t batMax = cu3BatMaxValue(); 

    // CU3 seems to calculate % with something close to:
    // floor( (val - (0.091 * max)) / 0.009 * max )
    // Below we do the reverse.
    uint32_t offsetK = (91 * batMax);
    uint32_t onePercentK = (9 * batMax);
    uint32_t valueK = offsetK + onePercentK * batPercentage + onePercentK / 2;
    return (uint16_t) (valueK / 1000);
}

/** 
 * Handle the messages that seem to be sent by the CU3 only.
 */
bool handleCu3Message(const messageType& message) {
    if(message.type == MSG_CMD_REQ && message.payloadSize == 2 && message.command == CMD_GET_DATA && message.payload[1] == 0x3b) {
        // GET DATA 083b 08:3b(Distance to maintenance)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], 0x00, 0x01, 0xE2, 0x08};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return true;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 2 && message.command == CMD_GET_DATA && message.payload[1] == 0x80) {
        // GET DATA 0880 08:80(Total distance)
        uint32_t total = getTotal();
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], FROM_UINT32(total)};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return true;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 2 && message.command == CMD_GET_DATA && message.payload[1] == 0x8e) {
        // GET DATA 088e 08:8e(Time)
        int64_t seconds = esp_timer_get_time() / (1000 * 1000) + offset;
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], FROM_UINT32(seconds)};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return true;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 2 && message.command == CMD_GET_DATA && message.payload[1] == 0x18) {
        // GET DATA 1418 14:18(Battery level)
        uint16_t batVal = toCu3BatValue(getBatPercentage());
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], FROM_UINT16(batVal)};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return true;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 4 && message.command == CMD_GET_DATA && message.payload[1] == 0x18 && message.payload[3] == 0x1a) {
        // GET DATA 9418141a 14:18(Battery level) 14:1a(Max battery level)
        uint16_t batVal = toCu3BatValue(getBatPercentage());
        uint16_t batMax = cu3BatMaxValue();
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], FROM_UINT16(batVal), message.payload[2], message.payload[3], FROM_UINT16(batMax)};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return true;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 2 && message.command == CMD_GET_DATA && message.payload[1] == 0x94) {
        // GET DATA 2894 28:94(Unknown)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], 0x40, 0x0e, 0x14, 0x7b};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return true;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 3 && message.command == CMD_GET_DATA && message.payload[1] == 0x9a && message.payload[2] == 0x00) {
        // GET DATA 449a00 44:9a[0](Max speed)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], 0x02, 0x00, 0x00, 0x00, 0xd0};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return true;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 3 && message.command == CMD_GET_DATA && message.payload[1] == 0x99 && message.payload[2] == 0x00) {
        // GET DATA 489900 48:99[0](Trip time)
        uint8_t payload[] = {0x00, message.payload[0], message.payload[1], 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf6};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return true;
    } else if(message.type == MSG_CMD_REQ && message.payloadSize == 6 && message.command == CMD_PUT_DATA && message.payload[1] == 0x8e) {
        // PUT DATA 8e, (Time)
        uint32_t newTime = toUint32(message.payload, 2);
        int64_t seconds = esp_timer_get_time() / (1000 * 1000);
        offset = newTime - seconds;

        uint8_t payload[] = {0x00};
        writeMessage(cmdResp(message.source, MSG_BMS, message.command, payload, sizeof(payload)));
        return true;
    }
    return false;
}

#endif
