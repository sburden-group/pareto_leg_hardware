#ifndef ANALOG_INPUT_H
#define ANALOG_INPUT_H

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>

void setup_adc(uint32_t ADCx);
void setup_gpio_for_adc(uint32_t gpio_port, uint8_t adc_input_channel);

uint16_t read_adc_naiive(uint32_t adcx, uint8_t adc_in_channel);




#endif // ANALOG_INPUT_H
