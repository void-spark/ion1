#include "storage.h"
#include "nvs_flash.h"
#include "nvs.h"
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

uint8_t *calibrationLoad(void)
{
    static uint8_t payload[11];

    // Byte 0 is altijd 0x00
    payload[0] = 0x00;

    // Fallback calibratie (10 bytes)
    static const uint8_t fallback[10] = {
        0x94, 0x38, 0x4b, 0x15, 0x28, 0x3a, 0x3e, 0x91, 0x79, 0x50
    };

    // Probeer calibratie uit NVS te lezen
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);

    if (err == ESP_OK) {
        size_t size = 10;
        err = nvs_get_blob(handle, NVS_KEY_CALIB, payload + 1, &size);
        nvs_close(handle);

        if (err == ESP_OK) {
            return payload;    // Succesvol geladen
        }
    }

    // Geen calibratie ? fallback
    memcpy(payload + 1, fallback, 10);
    return payload;
}

bool calibrationSave(uint8_t *source)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
        return false;

    esp_err_t err = nvs_set_blob(handle, NVS_KEY_CALIB, source, 10);
    if (err != ESP_OK) {
        nvs_close(handle);
        return false;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    return (err == ESP_OK);
}