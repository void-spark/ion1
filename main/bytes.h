#pragma once

#include <sys/unistd.h>

uint16_t toUint16(const uint8_t *buffer, size_t offset);
uint32_t toUint32(const uint8_t *buffer, size_t offset);
void fromUint16(const uint16_t value, uint8_t *buffer, size_t offset);
void fromUint32(const uint32_t value, uint8_t *buffer, size_t offset);

#define FROM_UINT16(value) (uint8_t)(value >> 8), (uint8_t)(value >> 0)
#define FROM_UINT32(value) (uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)(value >> 0)