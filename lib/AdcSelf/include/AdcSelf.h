#ifndef AdcSelf_h
#define AdcSelf_h
#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

#define DEFAULT_VREF 1100U

class AdcSelf {
  public:
    typedef enum {
        ADC1_CH0,
        ADC1_CH1,
        ADC1_CH2,
        ADC1_CH3,
        ADC1_CH4,
        ADC2_CH0,
        sizeof_ADC
    } channel_t;

    typedef enum {
        ATTEN_0db,
        ATTEN_2_5db,
        ATTEN_6db,
        ATTEN_11db,
        ATTEN_MAX
    } attenuation_t;

    bool attachPin(channel_t pin, attenuation_t attenuation = ATTEN_11db);

    uint16_t read(channel_t pin);
    uint32_t readMilliVolts(channel_t pin);

  private:
    void operator=(AdcSelf const &other) = delete;

    bool __initialized = false;

    uint16_t __analogVRef = 0U;
    adc_bits_width_t __width = (adc_bits_width_t)(ADC_WIDTH_MAX - 1U);
    uint8_t __clockDiv = 1U;
    attenuation_t __pinAttenuation[SOC_GPIO_PIN_COUNT];

    void init();
    void setPinAttenuation(channel_t pin, attenuation_t attenuation);
};

#endif // AdcSelf_h