/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
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
#include "stts75.h"

#include "syscfg.h"

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
	rcc_periph_clock_enable(LED_DISCO_RCC);
	gpio_set_mode(LED_DISCO_BLUE_PORT, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, LED_DISCO_BLUE_PIN);
	gpio_set_mode(LED_DISCO_GREEN_PORT, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, LED_DISCO_GREEN_PIN);
}

static void i2c_setup(void)
{

	/* Set alternate functions for the SCL and SDA pins of I2C2. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		GPIO_I2C2_SCL | GPIO_I2C2_SDA);

	/* Disable the I2C before changing any configuration. */
	i2c_peripheral_disable(I2C2);

	/* APB1 is running at 36MHz. */
	i2c_set_clock_frequency(I2C2, I2C_CR2_FREQ_36MHZ);

	/* 400KHz - I2C Fast Mode */
	i2c_set_fast_mode(I2C2);

	/*
	 * fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
	 * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
	 * Datasheet suggests 0x1e.
	 */
	i2c_set_ccr(I2C2, 0x1e);

	/*
	 * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
	 * 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
	 * Incremented by 1 -> 11.
	 */
	i2c_set_trise(I2C2, 0x0b);

	/*
	 * This is our slave address - needed only if we want to receive from
	 * other masters.
	 */
	i2c_set_own_7bit_slave_address(I2C2, 0x32);

	/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(I2C2);
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
	uint16_t temperature;

	clock_setup();
	led_setup();
	usart_setup();
	i2c_setup();

	gpio_clear(LED_DISCO_BLUE_PORT, LED_DISCO_BLUE_PIN);
	gpio_set(LED_DISCO_GREEN_PORT, LED_DISCO_GREEN_PIN);

	printf("Hello from: " __FILE__ "\n");

	stts75_write_config(I2C2, STTS75_SENSOR0);
	stts75_write_temp_os(I2C2, STTS75_SENSOR0, 0x1a00); /* 26 degrees */
	stts75_write_temp_hyst(I2C2, STTS75_SENSOR0, 0x1a00);
	temperature = stts75_read_temperature(I2C2, STTS75_SENSOR0);

	/* Send the temperature as binary over USART1. */
	for (i = 15; i >= 0; i--) {
		if (temperature & (1 << i))
			usart_send(USART1, '1');
		else
			usart_send(USART1, '0');
	}

	usart_send(USART1, '\r');
	usart_send(USART1, '\n');

	gpio_clear(GPIOB, GPIO6); /* LED2 on */

	while (1); /* Halt. */

	return 0;
}
