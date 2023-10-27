#ifndef defines_h
#define defines_h
#include <AdcSelf.h>

/*Channels for PWM*/
#define RED_LED_CHANNEL 0U
#define GREEN_LED_CHANNEL 1U

#define ADC1_CH0_PIN ADC1_CH0   /*GPIO0*/
#define ADC1_CH1_PIN ADC1_CH1   /*GPIO1*/
#define ADC1_CH2_PIN ADC1_CH2   /*GPIO2*/
#define WAKEUP_PIN ADC1_CH2_PIN /*GPIO2*/
#define ADC1_CH3_PIN ADC1_CH3   /*GPIO3*/
#define ADC1_CH4_PIN ADC1_CH4   /*GPIO4*/
#define ADC2_CH0_PIN ADC2_CH0   /*GPIO5*/
#define GATE_0_PIN 6U           /*GPIO6*/
#define GATE_1_PIN 7U           /*GPIO7*/
#define GATE_2_PIN 8U           /*GPIO8*/
#define CLK_PIN 10U             /*GPIO10*/
#define RED_LED_PIN 12U         /*SPIHD*/
#define GREEN_LED_PIN 13U       /*SPIWP*/
#define USB_N_PIN 18U           /*GPIO18*/
#define USB_P_PIN 19U           /*GPIO19*/
#define PUMP_PIN 20U            /*U0RX*/
#define ONEWIRE_PIN 21U         /*U0TX*/

#if (DEBUG == 1)
#define DEBUG_PIN 9U /*GPIO9*/
#endif               // DEBUG

#endif // defines_h