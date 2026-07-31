#include "storage.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <string.h>     // <-- toegevoegd voor memcpy()

static const char *TAG = "storage";

#define NVS_NAMESPACE "storage"
#define NVS_KEY_BATDATA "batdata"
#define NVS_KEY_CALIB   "calibration"

// Centrale instantie van batData
static struct batData bat;

// -----------------------------------------------------------------------------
// Initialisatie
// -----------------------------------------------------------------------------

void storageInit(void)
{
    // NVS init
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Defaults voor batData
    bat.trip1 = 0;
    bat.trip2 = 0;
    bat.total = 0;

    bat.percentage = 0;
    bat.mv = 0;
    bat.mah = 0;

    ESP_LOGI(TAG, "Storage initialized");
}

// -----------------------------------------------------------------------------
// batData API
// -----------------------------------------------------------------------------

struct batData *batDataGet(void)
{
    return &bat;
}

bool batDataLoad(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
        return false;

    size_t size = sizeof(bat);
    esp_err_t err = nvs_get_blob(handle, NVS_KEY_BATDATA, &bat, &size);
    nvs_close(handle);

    return (err == ESP_OK);
}

bool batDataSave(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
        return false;

    esp_err_t err = nvs_set_blob(handle, NVS_KEY_BATDATA, &bat, sizeof(bat));
    if (err != ESP_OK) {
        nvs_close(handle);
        return false;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    return (err == ESP_OK);
}

// -----------------------------------------------------------------------------
// Calibration API
// -----------------------------------------------------------------------------

bool calibrationLoad(void *out_value, size_t length) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    esp_err_t err = nvs_get_blob(handle, NVS_KEY_CALIB, out_value, &length);
    nvs_close(handle);

    return (err == ESP_OK);
}

bool calibrationSave(const void *value, size_t length) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }

    esp_err_t err = nvs_set_blob(handle, NVS_KEY_CALIB, value, length);
    if (err != ESP_OK) {
        nvs_close(handle);
        return false;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    return (err == ESP_OK);
}