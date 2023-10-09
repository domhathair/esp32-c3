#include "AdcSelf.h"

void AdcSelf::init() {
    if (__initialized) {
        return;
    }
    __initialized = true;
    for (unsigned counter = 0; counter < SOC_GPIO_PIN_COUNT; counter++) {
        __pinAttenuation[counter] = ATTEN_MAX;
    }
}

bool AdcSelf::attachPin(channel_t pin, attenuation_t attenuation) {
    int8_t channel = digitalPinToAnalogChannel(pin);
    if (channel < 0) {
        log_e(">> Pin %u is not ADC pin!", (uint8_t)pin);
        return false;
    }
    /*pinMode(pin, ANALOG);*/
    setPinAttenuation(pin, attenuation);
    return true;
}

uint16_t AdcSelf::read(channel_t pin) {
    int8_t channel = digitalPinToAnalogChannel(pin);
    int value = 0;
    esp_err_t error = ESP_OK;
    if (channel < 0) {
        log_e(">> Pin %u is not ADC pin!", pin);
        return value;
    }
    if (channel > (SOC_ADC_MAX_CHANNEL_NUM - 1)) {
        channel -= SOC_ADC_MAX_CHANNEL_NUM;
        error = adc2_get_raw((adc2_channel_t)channel, (adc_bits_width_t)__width,
                             &value);
        if (error == ESP_ERR_INVALID_STATE) {
            log_e(">> GPIO%u: %s: ADC2 not initialized yet.", pin,
                  esp_err_to_name(error));
        } else if (error == ESP_ERR_TIMEOUT) {
            log_e(">> GPIO%u: %s: ADC2 is in use by Wi-Fi.", pin,
                  esp_err_to_name(error));
        } else if (error != ESP_OK) {
            log_e(">> GPIO%u: %s.", pin, esp_err_to_name(error));
        }
    } else {
        value = adc1_get_raw((adc1_channel_t)channel);
    }
    return value;
}

uint32_t AdcSelf::readMilliVolts(channel_t pin) {
    int8_t channel = digitalPinToAnalogChannel(pin);
    if (channel < 0) {
        log_e(">> Pin %u is not ADC pin!", pin);
        return 0;
    }

    if (!__analogVRef) {
        if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
            log_e(">> eFuse Two Point: Supported.");
            __analogVRef = DEFAULT_VREF;
        }
        if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
            log_e(">> eFuse Vref: Supported.");
            __analogVRef = DEFAULT_VREF;
        }
        if (!__analogVRef) {
            __analogVRef = DEFAULT_VREF;
        }
    }
    adc_unit_t unit = ADC_UNIT_1;
    if (channel > (SOC_ADC_MAX_CHANNEL_NUM - 1)) {
        unit = ADC_UNIT_2;
    }

    uint16_t adc_reading = read(pin);

    esp_adc_cal_characteristics_t chars = {};
    esp_adc_cal_value_t val_type =
        esp_adc_cal_characterize(unit, (adc_atten_t)__pinAttenuation[pin],
                                 __width, __analogVRef, &chars);

    static bool print_chars_info = true;
    if (print_chars_info) {
        if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
            log_e("ADC%u: Characterized using Two Point Value: %u.", unit,
                  chars.vref);
        } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
            log_e("ADC%u: Characterized using eFuse Vref: %u.", unit,
                  chars.vref);
        } else {
            log_e("ADC%u: Characterized using Default Vref: %u.", unit,
                  chars.vref);
        }
        print_chars_info = false;
    }
    return esp_adc_cal_raw_to_voltage((uint32_t)adc_reading, &chars);
}

void AdcSelf::setPinAttenuation(channel_t pin, attenuation_t attenuation) {
    int8_t channel = digitalPinToAnalogChannel(pin);
    if (channel < 0) {
        return;
    }
    if (attenuation > ATTEN_MAX) {
        attenuation = ATTEN_MAX;
    }
    init();
    __pinAttenuation[pin] = attenuation;
    if (channel > (SOC_ADC_MAX_CHANNEL_NUM - 1)) {
        adc2_config_channel_atten(
            (adc2_channel_t)(channel - SOC_ADC_MAX_CHANNEL_NUM),
            (adc_atten_t)__pinAttenuation[pin]);
    } else {
        adc1_config_channel_atten((adc1_channel_t)channel,
                                  (adc_atten_t)__pinAttenuation[pin]);
    }
}