#include <string.h>
#include "cmds.h"

messageType message(uint8_t target, uint8_t type, uint8_t source, uint8_t command, uint8_t *payload, size_t payloadSize) {
    messageType result = {};
    result.target = target;
    result.type = type;
    result.source = source;
    result.command = command;
    memcpy(result.payload, payload, payloadSize);
    result.payloadSize = payloadSize;

    return result;
}

messageType message(uint8_t target, uint8_t type, uint8_t source, uint8_t command) {
    return message(target, type, source, command, NULL, 0);
}

messageType message(uint8_t target, uint8_t type, uint8_t source) {
    return message(target, type, source, 0x00);
}

messageType message(uint8_t target, uint8_t type) {
    return message(target, type, 0x00);
}

messageType handoffMsg(uint8_t target) {
    return message(target, MSG_HANDOFF);
}

messageType pingReq(uint8_t target, uint8_t source) {
    return message(target, MSG_PING_REQ, source);
}

messageType pingResp(uint8_t target, uint8_t source) {
    return message(target, MSG_PING_RESP, source);
}

messageType cmdReq(uint8_t target, uint8_t source, uint8_t command) {
    return message(target, MSG_CMD_REQ, source, command);
}

messageType cmdResp(uint8_t target, uint8_t source, uint8_t command) {
    return message(target, MSG_CMD_RESP, source, command);
}

messageType cmdReq(uint8_t target, uint8_t source, uint8_t command, uint8_t *payload, size_t payloadSize) {
    return message(target, MSG_CMD_REQ, source, command, payload, payloadSize);
}

messageType cmdResp(uint8_t target, uint8_t source, uint8_t command, uint8_t *payload, size_t payloadSize) {
    return message(target, MSG_CMD_RESP, source, command, payload, payloadSize);
}
