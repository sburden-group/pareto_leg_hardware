#ifndef SYSTEM_MILLIS_H
#define SYSTEM_MILLIS_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

static const uint32_t DEFAULT_CLK_FREQ = 216000;

extern volatile uint32_t system_millis;

void setup_system_clock();
void setup_systick();
void delay_ms(uint32_t delay);
void millis_init();

#endif //SYSTEM_MILLIS_H
