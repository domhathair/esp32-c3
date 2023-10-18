#ifndef AdcSelf_h
#define AdcSelf_h
#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

#define DEFAULT_VREF 1100U
#define ADC1_CH0 0U
#define ADC1_CH1 1U
#define ADC1_CH2 2U
#define ADC1_CH3 3U
#define ADC1_CH4 4U
#define ADC2_CH0 5U
#define sizeof_ADC 6U

#define ATTEN_0db 0U
#define ATTEN_2_5db 1U
#define ATTEN_6db 2U
#define ATTEN_11db 3U
#define ATTEN_MAX 4U

class AdcSelf {
  public:
    bool attachPin(uint8_t pin, uint8_t attenuation = ATTEN_11db);

    uint16_t read(uint8_t pin);
    uint32_t readMilliVolts(uint8_t pin);

  private:
    void operator=(AdcSelf const &other) = delete;

    bool __initialized = false;

    uint16_t __analogVRef = 0U;
    adc_bits_width_t __width = (adc_bits_width_t)(ADC_WIDTH_MAX - 1U);
    uint8_t __clockDiv = 1U;
    uint8_t __pinAttenuation[SOC_GPIO_PIN_COUNT];

    void init();
    void setPinAttenuation(uint8_t pin, uint8_t attenuation);
};

#endif // AdcSelf_h