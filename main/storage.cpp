#include <sys/stat.h>
#include "esp_log.h"
#include "esp_spiffs.h"
#include "storage.h"

static const char *TAG = "storage";

#define CALIBRATION_FILE "/spiffs/calibration.bin"

void init_spiffs() {
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t spiffs_conf = {};
    spiffs_conf.base_path = "/spiffs";
    spiffs_conf.partition_label = NULL;
    spiffs_conf.max_files = 5;
    spiffs_conf.format_if_mount_failed = true;

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_conf));

    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info(spiffs_conf.partition_label, &total, &used);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    
    struct stat st;
    if(stat(CALIBRATION_FILE, &st) == 0) {
        FILE *fp = fopen(CALIBRATION_FILE, "r");
        if(fp == NULL) {
            ESP_LOGE(TAG, "Failed to open calibration file for reading");
        } else {
            uint8_t data[10];
            size_t read = fread(data, 1, sizeof(data), fp);
            fclose(fp);
            ESP_LOGI(TAG, "Calibration file found. Size: %lu, content:", st.st_size);        
            ESP_LOG_BUFFER_HEX(TAG, data, read);
        }
    }
}

bool calibrationFileExists() {
    struct stat st;
    return stat(CALIBRATION_FILE, &st) == 0;
}

bool readCalibrationData(uint8_t * target) {
    FILE *fp = fopen(CALIBRATION_FILE, "r");
    if(fp == NULL) {
        ESP_LOGE(TAG, "Failed to open calibration file for reading");
        return false;
    }
    fread(target, 1, 10, fp);
    fclose(fp);

    return true;
}

bool writeCalibrationData(uint8_t * source){
    FILE *fp = fopen(CALIBRATION_FILE, "w");
    if(fp == NULL) {
        ESP_LOGE(TAG, "Failed to open calibration file for writing");
        return false;
    }
    fwrite(source, 1, 10, fp);
    fclose(fp);

    return true;
}

