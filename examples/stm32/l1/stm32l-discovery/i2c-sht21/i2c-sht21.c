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
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

/* LEDS on stm32L discovery board conflict with I2C1 on primary pins !*/
#define LED_DISCO_GREEN_PORT GPIOB
#define LED_DISCO_BLUE_PORT GPIOB
#define LED_DISCO_GREEN_PIN GPIO7
#define LED_DISCO_BLUE_PIN GPIO6

#define USART_CONSOLE USART2

#define SENSOR_ADDRESS (0x40)

enum sht21_cmd_e {
	SHT21_CMD_TEMP_HOLD = 0xe3,
	SHT21_CMD_HUMIDITY_HOLD = 0xe5,
	SHT21_CMD_TEMP_NOHOLD = 0xf3,
	SHT21_CMD_HUMIDITY_NOHOLD = 0xf5,
	SHT21_CMD_WRITE_REG = 0xe6,
	SHT21_CMD_READ_REG = 0xe7,
	SHT21_CMD_RESET = 0xfe,
	/* 0xfa, 0x0f to read serial */
};


int _write(int file, char *ptr, int len);

static void clock_setup(void)
{
	rcc_clock_setup_pll(&clock_config[CLOCK_VRANGE1_HSI_PLL_24MHZ]);
	rcc_periph_clock_enable(RCC_GPIOB);
}

static void usart_setup(void)
{
	/* Enable clocks for USART2 peripheral */
	rcc_periph_clock_enable(RCC_USART2);
	/* usart pins */
	rcc_periph_clock_enable(RCC_GPIOA);
	/* Setup GPIO pins for USART2 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);

	/* Setup USART2 TX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);

	usart_set_baudrate(USART_CONSOLE, 115200);
	usart_set_databits(USART_CONSOLE, 8);
	usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
	usart_set_mode(USART_CONSOLE, USART_MODE_TX);
	usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
	usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART_CONSOLE);
}

/**
 * Use USART_CONSOLE as a console.
 * This is a syscall for newlib
 * @param file the fd being written to
 * @param ptr data to write
 * @param len how many bytes to write
 * @return should return the number of bytes written, or -1 and set errno
 */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART_CONSOLE, '\r');
			}
			usart_send_blocking(USART_CONSOLE, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

static void codec_gpio_init(void)
{
	/* reset pin */
#if 0
	rcc_periph_clock_enable(RCC_GPIOD);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
#endif

	/* i2c control lines */
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_10MHZ, GPIO8 | GPIO9);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO8 | GPIO9);
	/* sht21 needs ~15ms to bootup */
	int i, j;
	gpio_set(LED_DISCO_BLUE_PORT, LED_DISCO_BLUE_PIN);
	for (i = 0; i < 50; i++) { /* Wait a bit. */
		for (j = 0; j < 100000; j++) { /* Wait a bit. */
			__asm__("NOP");
		}
	}
	gpio_clear(LED_DISCO_BLUE_PORT, LED_DISCO_BLUE_PIN);
}

static void codec_i2c_init(void)
{
	rcc_periph_clock_enable(RCC_I2C1);
	i2c_peripheral_disable(I2C1);
	i2c_reset(I2C1);
	i2c_set_standard_mode(I2C1);
	//i2c_enable_ack(I2C1);
	i2c_set_dutycycle(I2C1, I2C_CCR_DUTY_DIV2); /* default, no need to do this really */

	/* Board specific settings based on time */
	i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_24MHZ);
	/* 24MHz / (100kHz * 2) */
	i2c_set_ccr(I2C1, 120);
	/* standard mode, freqMhz+1*/
	i2c_set_trise(I2C1, 25);
	/* End of board specific settings */

	i2c_peripheral_enable(I2C1);
}

static void codec_init(void)
{
	/* Configure the Codec related IOs */
	codec_gpio_init();

	codec_i2c_init();
}

static int codec_write_reg(uint8_t reg, uint8_t val)
{
	uint32_t i2c = I2C1;

	while ((I2C_SR2(i2c) & I2C_SR2_BUSY)) {
	}

	i2c_send_start(i2c);

	/* Wait for master mode selected */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, SENSOR_ADDRESS, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	uint32_t reg32 = I2C_SR2(i2c);
	(void) reg32; /* unused */

	/* Common above here */

	/* Sending the data. */
	i2c_send_data(i2c, reg);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));
	i2c_send_data(i2c, val);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	/* Send STOP condition. */
	i2c_send_stop(i2c);
	return 0;
}

static uint32_t codec_read_reg(uint8_t reg)
{
	uint32_t i2c = I2C1;

	while ((I2C_SR2(i2c) & I2C_SR2_BUSY)) {
	}

	i2c_send_start(i2c);

	/* Wait for master mode selected */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, SENSOR_ADDRESS, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	uint32_t reg32 = I2C_SR2(i2c);
	(void) reg32; /* unused */

	/*  Common stuff ABOVE HERE     */

	i2c_send_data(i2c, reg);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));

	i2c_send_start(i2c);

	/* Wait for master mode selected */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, SENSOR_ADDRESS, I2C_READ);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	//i2c_disable_ack(i2c);

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);
	(void) reg32; /* unused */

	//i2c_send_stop(i2c);

	while (!(I2C_SR1(i2c) & I2C_SR1_RxNE));
	uint32_t result = i2c_get_data(i2c);

	//i2c_enable_ack(i2c);
	I2C_SR1(i2c) &= ~I2C_SR1_AF;
	return result;
}

static void codec_readid(void)
{
	uint8_t raw = codec_read_reg(SHT21_CMD_READ_REG);
	printf("raw user reg = %#x\n", raw);
	int res = ((raw & 0x80) >> 6) | (raw & 1);
	printf("temp resolution is in %d bits\n", 14 - res);
	printf("battery status: %s\n", (raw & (1<<6) ? "failing" : "good"));
	printf("On chip heater: %s\n", (raw & 0x2) ? "on" : "off");
}

int main(void)
{
	int i, j;
	clock_setup();
	gpio_mode_setup(LED_DISCO_GREEN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_DISCO_GREEN_PIN);
	gpio_mode_setup(LED_DISCO_BLUE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_DISCO_BLUE_PIN);
	gpio_set(LED_DISCO_GREEN_PORT, LED_DISCO_GREEN_PIN);
	usart_setup();
	printf("hi guys! this is: " __FILE__ "\n");
	/* green led for ticking */

	codec_init();
	codec_readid();

#if 0
	codec_write_reg(0x14, 0xff);
	for (i = 0; i < 8; i++) {
		uint8_t pass_vol_a = codec_read_reg(0x14);
		printf("Passthrough vol A was: %#x\n", pass_vol_a);
		codec_write_reg(0x14, pass_vol_a >> 1);
		gpio_toggle(LED_DISCO_GREEN_PORT, LED_DISCO_GREEN_PIN);
		for (j = 0; j < 100000; j++) { /* Wait a bit. */
			__asm__("NOP");
		}
	}
#endif

	/* Nothing else to do */;
	while (1) {
		;
	}
	return 0;
}
