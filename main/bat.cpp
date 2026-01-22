#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "bat.h"

static const char *TAG = "bat";

static uint32_t chargeFullMah = (CONFIG_ION_BAT_CHARGE * 3);

// ADC Attenuation
// 11DB = 3.55 voltage gain, reference voltage should be around 1100mv,
// so max theoretical measurement would be 3905mv, actual/recommended(?) is a lot lower.
#define ADC_ATTEN ADC_ATTEN_DB_11

static bool cali_enable = false;
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;

// Batterij en stroomwaarden
// Use a fake value of CONFIG_ION_ADC_FULL_MVv when we don't have ADC.
static uint32_t batMv = 27600;
static uint32_t batMa = 0;
static uint32_t historyMa = 0;

static uint32_t history;
static uint8_t batPercentage;

// Get lower/upper limit from configuration
static uint32_t emptyMv = CONFIG_ION_ADC_EMPTY_MV;
static uint32_t fullMv = CONFIG_ION_ADC_FULL_MV;
static void adc_calibration_init(adc_unit_t unit, adc_atten_t atten) {
    esp_err_t ret = ESP_FAIL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!cali_enable) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {};
        cali_config.unit_id = unit;
        cali_config.atten = atten;
        cali_config.bitwidth = ADC_BITWIDTH_DEFAULT;
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);
        if (ret == ESP_OK) {
            cali_enable = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!cali_enable) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {};
        cali_config.unit_id = unit;
        cali_config.atten = atten;
        cali_config.bitwidth = ADC_BITWIDTH_DEFAULT;
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle);
        if (ret == ESP_OK) {
            cali_enable = true;
        }
    }
#endif

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !cali_enable) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }
}

static void adc_calibration_deinit() {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(adc1_cali_handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(adc1_cali_handle));
#endif
}

void adc_init() {

    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {};
    config.atten = ADC_ATTEN;
    config.bitwidth = ADC_BITWIDTH_DEFAULT;

    // Voltage channel
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, (adc_channel_t)CONFIG_ION_ADC_CHAN, &config));

    // Current Channel
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, (adc_channel_t)CONFIG_ION_CURR_ADC_CHAN, &config));

    adc_calibration_init(ADC_UNIT_1, ADC_ATTEN);
}

uint32_t measureBatMv() {
    int adc_raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, (adc_channel_t)CONFIG_ION_ADC_CHAN, &adc_raw));
    int adcVoltageMv = 0;
    if (cali_enable) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &adcVoltageMv));
    } else {
        // ADC_BITWIDTH_DEFAULT means use max bits, which is SOC_ADC_RTC_MAX_BITWIDTH
        // 11DB is a factor 3.55, and base reference voltage is 1V (I think)
        adcVoltageMv = (adc_raw * 3550) / (1 << SOC_ADC_RTC_MAX_BITWIDTH);
    }

    // Calculate actual voltage in mv
    return (adcVoltageMv * CONFIG_ION_DIVIDER_SCALE) / 1000;
}

uint32_t measureCurrentMv() {
    int adc_raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, (adc_channel_t)CONFIG_ION_CURR_ADC_CHAN, &adc_raw));
    int adcCurrentMv = 0;
    if (cali_enable) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &adcCurrentMv));
    } else {
        adcCurrentMv = (adc_raw * 3550) / (1 << SOC_ADC_RTC_MAX_BITWIDTH);
    }
	historyMa += adcCurrentMv;
	uint32_t avg = historyMa >> 5;
	historyMa -= avg;

    return avg;
}

static uint8_t batMvToPercentage(uint32_t batMv) {

    // Calculate the percentage
    uint32_t percentage = (batMv < emptyMv) ? 0 : ((batMv - emptyMv) * 100) / (fullMv - emptyMv);

    // Limit to 0-100
    uint8_t batterypercentage = 0;
    if (percentage > 100) {
        batterypercentage = 100;
    } else {
        batterypercentage = (uint8_t)percentage;
    }

    return batterypercentage;
}

void measureBat() {
    batMv = measureBatMv();

	// This is provided by 'mooiweertje' and is pretty much similar to Simple Exponential Smoothing (https://en.wikipedia.org/wiki/Exponential_smoothing).
	// By using an alpha of 1/128, and storing the smoothed value scaled by 128 in history, this can be written very efficiently though,
	// and the scaled value allows us to work with integers instead of floating point.
	// It should take about 5 x 128 (640) calls to settle on a value (at 99.3%), and we try to measure every 100ms,
	// which puts us a bit over 60 seconds. That's quite slow, but for a battery indicator should be ok.
	history += batMv;
	uint32_t avg = history >> 7;
	history -= avg;

    batPercentage = batMvToPercentage(avg);
}

void measureCurrent() {
    batMa = measureCurrentMv();
}

uint32_t getBatMv() {
    return batMv;
}

uint8_t getBatPercentage() {
    if(batMv == 0) {
        // Use a fake value of 50% when we don't have ADC.
        return 50;
    }

    return batPercentage;
}

uint32_t getBatMa() {
    return batMa;
}

void adc_teardown() {

    // Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (cali_enable) {
        adc_calibration_deinit();
    }
}
