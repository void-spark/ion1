
#include "bytes.h"

uint16_t toUint16(const uint8_t *buffer, size_t offset) { 
    return ((uint16_t)buffer[offset] << 8) | ((uint16_t)buffer[offset + 1] << 0); 
}

uint32_t toUint32(const uint8_t *buffer, size_t offset) {
    return ((uint32_t)buffer[offset] << 24) | ((uint32_t)buffer[offset + 1] << 16) | ((uint32_t)buffer[offset + 2] << 8) | ((uint32_t)buffer[offset + 3] << 0);
}

void fromUint16(const uint16_t value, uint8_t *buffer, size_t offset) {
        buffer[offset + 0] = (value >> 8) & 0xff;
        buffer[offset + 1] = (value >> 0) & 0xff;
}

void fromUint32(const uint32_t value, uint8_t *buffer, size_t offset) {
        buffer[offset + 0] = (value >> 24) & 0xff;
        buffer[offset + 1] = (value >> 16) & 0xff; 
        buffer[offset + 2] = (value >> 8) & 0xff;
        buffer[offset + 3] = (value >> 0) & 0xff;
}
