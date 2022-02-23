#include <millis.h>

volatile uint32_t system_millis;

void setup_system_clock()
{
    // Set internal clock to run at 168[MHz] using 25[MHz] source.
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}

void setup_systick()
{
    //int((clock_rate - 1)/1000) to get 1ms interrupt.
    systick_set_reload(DEFAULT_CLK_FREQ);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    // Do this last.
    systick_interrupt_enable();
}


// This fn is declared in nvic.h
void sys_tick_handler(void)
{
    system_millis++;
}

void delay_ms(uint32_t delay)
{
    uint32_t wake_time = delay + system_millis;
    while (wake_time - system_millis > 0); // Do nothing.
}

void millis_init()
{
    system_millis = 0;
    setup_system_clock();
    setup_systick();
}

