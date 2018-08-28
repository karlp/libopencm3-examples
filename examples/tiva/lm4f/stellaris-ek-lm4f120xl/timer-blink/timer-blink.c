/*
 * This file is part of the libopencm3 project.
 *
 * Copyright 2018 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/lm4f/gpio.h>
#include <libopencm3/lm4f/rcc.h>
#include <libopencm3/lm4f/systemcontrol.h>
#include <libopencm3/lm4f/timer.h>

#define LED_PORT GPIOF
#define LED_PIN_R GPIO1 
#define LED_PIN_B GPIO2
#define LED_PIN_G GPIO3
#define LED_PINS (LED_PIN_R | LED_PIN_G | LED_PIN_B)

// f1 is T0CCP1
// f2 is T1CCP0
// f3 is T1CCP1  all are afsel 7

static void setup_timers(void)
{
	periph_clock_enable(RCC_TIMER0);
	periph_clock_enable(RCC_TIMER1);
	periph_clock_enable(RCC_GPIOF);
	
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PINS);
	gpio_set_output_config(LED_PORT, GPIO_OTYPE_PP, GPIO_DRIVE_2MA, LED_PINS);
	//gpio_set_af(LED_PORT, 7, LED_PINS);
	gpio_set_af(LED_PORT, 7, GPIO2);

	/* set this up for all eventually */

	/* pwm mode */
	/* ignore cfg reg, it's all defaults */

#define USE_16BIT_MODE 0
#if (USE_16BIT_MODE == 1)
	GPTMCFG(TIMER1_BASE) = 0x4; /* 16bit mode, apparently how you do pwm mode */
	GPTMTAMR(TIMER1_BASE) = (1<<3) | 2;
	GPTMTAPR(TIMER1_BASE) = 255; /* only in 16bit mode, now at 1e6 */
	GPTMTAILR(TIMER1_BASE) = 0xffff;
	GPTMTAMATCHR(TIMER1_BASE) = 0xf000; // 25% on or off, ish
	GPTMCTL(TIMER1_BASE) = (1<<6); /* inverted pwm */
	GPTMCTL(TIMER1_BASE) |= 1; /* turn on timer A */ 
#else
	/* 32bit mode, but only for timerA channels.. */
	GPTMCFG(TIMER1_BASE) = 0;
	GPTMTAMR(TIMER1_BASE) = (1<<3) | 2;
	//GPTMTAILR(TIMER1_BASE) = 320e6; /* 4 seconds */
	//GPTMTAMATCHR(TIMER1_BASE) = 160e6; /* 2 seconds */
	//GPTMCTL(TIMER1_BASE) = (1<<6); /* inverted pwm */
	GPTMTAILR(TIMER1_BASE) = 80e6; /* 1 seconds */
	GPTMTAMATCHR(TIMER1_BASE) = 40e6; /* 0.5 seconds */
	//GPTMCTL(TIMER1_BASE) = (1<<6); /* inverted pwm */
	
	GPTMCTL(TIMER1_BASE) |= 1; /* turn on timer A */ 
#endif
	//GPTMTnILR = 500, 1000, 3000; /* three rates for teh three leds */
}

int main(void)
{
	gpio_enable_ahb_aperture();
	/* 16MHz crystal, with 400MHz pll / 5 for 80MHz (max) */
	rcc_sysclk_config(OSCSRC_MOSC, XTAL_16M, 5);
	setup_timers();

	//gpio_set(GPIOF, GPIO1);
	while (1) {
		//gpio_toggle(GPIOF, GPIO1);
		for (int i = 0; i < 1000000; i++) {
			__asm__("nop");
		}
	}

	return 0;
}
