#include "storage.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "storage";

#define NVS_NAMESPACE "storage"

bool dataLoad(const char *key, void *out_value, size_t length) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    esp_err_t err = nvs_get_blob(handle, key, out_value, &length);
    nvs_close(handle);

    return (err == ESP_OK);
}

bool dataSave(const char *key, const void *value, size_t length) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }

    esp_err_t err = nvs_set_blob(handle, key, value, length);
    if (err != ESP_OK) {
        nvs_close(handle);
        return false;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    return (err == ESP_OK);
}