/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2014 Karl Palsson <karlp@tweak.net.au>
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
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>

#include "syscfg.h"
#include "sht21.h"

int _write(int file, char *ptr, int len);

static void clock_setup(void)
{
	rcc_clock_setup_in_hsi_out_24mhz();
	/* Enable clocks for console and i2c peripheral*/
	rcc_periph_clock_enable(CONSOLE_UART_RCC);
	rcc_periph_clock_enable(I2C_SENSOR_RCC);

	rcc_periph_clock_enable(RCC_AFIO);

	/* and IO Ports for the console, i2c, and leds */
	rcc_periph_clock_enable(CONSOLE_UART_RCC_GPIO);
	rcc_periph_clock_enable(LED_DISCO_RCC);
	rcc_periph_clock_enable(I2C_SENSOR_RCC_GPIO);
}

static void usart_setup(void)
{
	gpio_set_mode(CONSOLE_UART_GPIO_PORT, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, CONSOLE_UART_GPIO_PINS);

	/* Setup UART parameters. */
	usart_set_baudrate(CONSOLE_UART, 115200);
	usart_set_databits(CONSOLE_UART, 8);
	usart_set_stopbits(CONSOLE_UART, USART_STOPBITS_1);
	usart_set_mode(CONSOLE_UART, USART_MODE_TX);
	usart_set_parity(CONSOLE_UART, USART_PARITY_NONE);
	usart_set_flow_control(CONSOLE_UART, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(CONSOLE_UART);
}

static void led_setup(void)
{
	gpio_set_mode(LED_DISCO_BLUE_PORT, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, LED_DISCO_BLUE_PIN);
	gpio_set_mode(LED_DISCO_GREEN_PORT, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, LED_DISCO_GREEN_PIN);
}

static void i2c_setup(void)
{

	/* Set alternate functions for the SCL and SDA pins of I2C2. */
	gpio_set_mode(I2C_SENSOR_GPIO_PORT, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, I2C_SENSOR_GPIO_PINS);

	/* Disable the I2C before changing any configuration. */
	i2c_peripheral_disable(I2C_SENSOR);

	/* Thanks ST */
	i2c_set_clock_frequency(I2C_SENSOR, I2C_CR2_FREQ_24MHZ);
	//i2c_set_clock_frequency(I2C_SENSOR, rcc_ppre_frequency / 1000000);
	
	/* 100KHz - I2C Standard Mode */
	i2c_set_standard_mode(I2C_SENSOR);

	/* Who thought up this shit?! */
	/* 5uSec * freq MHz for standard mode */
	i2c_set_ccr(I2C_SENSOR, 120);

	/*
	 * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
	 * 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
	 * Incremented by 1 -> 11.
	 */
	/* fuck this shit too! 
	 for standard mode, this is freq in mhz + 1 ?!
	 for fast mode, this is freq 7.2 + 1 for 24, so 24 /3 + 1?!
	 */	
	i2c_set_trise(I2C_SENSOR, 25);
	
	/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(I2C_SENSOR);
}

static void i2c_errata_unfix(void) {
	i2c_peripheral_disable(I2C_SENSOR);
	gpio_set_mode(I2C_SENSOR_GPIO_PORT, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_OPENDRAIN, I2C_SENSOR_GPIO_PINS);
	gpio_set(I2C_SENSOR_GPIO_PORT, I2C_SENSOR_GPIO_PINS);
	/* FIXME - not abstracted to syscfg.h */
	gpio_clear(I2C_SENSOR_GPIO_PORT, GPIO_I2C1_SDA);
	gpio_clear(I2C_SENSOR_GPIO_PORT, GPIO_I2C1_SCL);
	gpio_set(I2C_SENSOR_GPIO_PORT, GPIO_I2C1_SCL);
	gpio_set(I2C_SENSOR_GPIO_PORT, GPIO_I2C1_SDA);
	gpio_set_mode(I2C_SENSOR_GPIO_PORT, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, I2C_SENSOR_GPIO_PINS);
	I2C_CR1(I2C_SENSOR) |= I2C_CR1_SWRST;
	I2C_CR1(I2C_SENSOR) &= ~I2C_CR1_SWRST;
	i2c_peripheral_enable(I2C_SENSOR);
}

/**
 * Use USART_CONSOLE as a console.
 * This is a syscall for newlib
 * @param file
 * @param ptr
 * @param len
 * @return
 */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(CONSOLE_UART, '\r');
			}
			usart_send_blocking(CONSOLE_UART, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

int main(void)
{
	int i = 0;
	float temperature;

	clock_setup();
	led_setup();
	usart_setup();
//	i2c_errata_unfix();
	i2c_setup();
	I2C_CR1(I2C_SENSOR) |= I2C_CR1_SWRST;
	I2C_CR1(I2C_SENSOR) &= ~I2C_CR1_SWRST;

	gpio_clear(LED_DISCO_BLUE_PORT, LED_DISCO_BLUE_PIN);
	gpio_set(LED_DISCO_GREEN_PORT, LED_DISCO_GREEN_PIN);

	printf("Hello from: " __FILE__ "\n");

	sht21_begin(I2C_SENSOR);
	
	while (1) {
		gpio_toggle(LED_DISCO_BLUE_PORT, LED_DISCO_BLUE_PIN);
		temperature = sht21_read_temperature();
		printf("Temperature (tick:%d): %f\n", i++, temperature);
	}

	return 0;
}
