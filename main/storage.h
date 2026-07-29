#pragma once
#include <stdint.h>
#include <stdbool.h>

bool dataLoad(const char *key, void *out_value, size_t length);
bool dataSave(const char *key, const void *value, size_t length);