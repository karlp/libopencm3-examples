/* 
 * Sets up a millisecond systick timer, 
 * and a millis() style timer to make things more compatible with arduino 
 * style code
 * 
 */


#include <stdint.h>
#include <libopencm3/cm3/nvic.h>
#include "ms_systick.h"


volatile uint64_t ksystick;

uint64_t millis(void)
{
    return ksystick;
}

/**
 * Busy loop for X ms USES INTERRUPTS
 * @param ms
 */
void delay_ms(unsigned int ms) {
    uint64_t now = millis();
    while (millis() - ms < now) {
        ;
    }
}

void sys_tick_handler(void)
{
    ksystick++;
}
