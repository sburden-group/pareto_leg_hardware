#include <analog_input.h>

void setup_adc(uint32_t adcx)
{
    switch (adcx)
    {
        case ADC1:
            // Setup Clocks.
            rcc_periph_clock_enable(RCC_ADC1);
            break;
        case ADC2:
            rcc_periph_clock_enable(RCC_ADC2);
            break;
        case ADC3:
            rcc_periph_clock_enable(RCC_ADC3);
            break;
    }
    // Set Common Settings:
    adc_power_off(adcx);
	adc_disable_scan_mode(adcx);
	adc_set_sample_time_on_all_channels(adcx, ADC_SMPR_SMP_3CYC);
	adc_power_on(adcx);
}

/**
 * setup GPIOs for analog input.
 */
void setup_gpio_for_adc(uint32_t gpio_port, uint8_t gpio_pin_mask)
{
    gpio_mode_setup(gpio_port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                    gpio_pin_mask);
}

/**
 * read the adc on a particular input channel.
 * Note: these must correspond with the right GPIO Pin. i.e: GPIOB0 is ADC1_IN8
 */
uint16_t read_adc_naiive(uint32_t adcx, uint8_t adc_in_channel)
{
	uint8_t channel_array[16];
	channel_array[0] = adc_in_channel;
	adc_set_regular_sequence(adcx, 1, channel_array);
	adc_start_conversion_regular(adcx);
	while (!adc_eoc(adcx)); // wait for end of conversion.
	uint16_t reg16 = adc_read_regular(adcx);
	return reg16;
}
