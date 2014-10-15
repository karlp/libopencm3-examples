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
#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
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

static void sht21_send_data(uint32_t i2c, size_t n, uint8_t *data) {
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

	size_t i;
	for (i = 0; i < n; i++) {
		i2c_send_data(i2c, data[i]);
		while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));
	}
}

static void sht21_send_cmd(uint32_t i2c, uint8_t cmd)
{
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

	i2c_send_data(i2c, cmd);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));
}

static uint32_t sht21_read1(uint32_t i2c)
{
	i2c_send_start(i2c);

	/* Wait for master mode selected */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, SENSOR_ADDRESS, I2C_READ);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	//i2c_disable_ack(i2c);  // SHould be here for single byte reception?

	/* Cleaning ADDR condition sequence. */
	uint32_t reg32 = I2C_SR2(i2c);
	(void) reg32; /* unused */


	while (!(I2C_SR1(i2c) & I2C_SR1_RxNE));
	uint32_t result = i2c_get_data(i2c);

	//i2c_enable_ack(i2c);
	I2C_SR1(i2c) &= ~I2C_SR1_AF;

	i2c_send_stop(i2c);
	//printf("%s returned %#" PRIX32 " (%" PRIu32 ")\n", __func__, result, result);
	return result;
}

static uint32_t sht21_read2(uint32_t i2c)
{
	i2c_send_start(i2c);

	/* Wait for master mode selected */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, SENSOR_ADDRESS, I2C_READ);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	i2c_disable_ack(i2c);
	i2c_nack_next(i2c);

	/* Cleaning ADDR condition sequence. */
	uint32_t reg32 = I2C_SR2(i2c);
	(void) reg32; /* unused */

	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));

	i2c_send_stop(i2c);
	uint32_t result = i2c_get_data(i2c);
	result <<= 8;
	result |= i2c_get_data(i2c);
	//printf("%s returned %#" PRIX32 " (%" PRIu32 ")\n", __func__, result, result);
	return result;
}

static void sht21_readn(uint32_t i2c, int n, uint8_t *res)
{
	//assert(n > 0);
	i2c_send_start(i2c);

	i2c_enable_ack(i2c);
	
	/* Wait for master mode selected */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, SENSOR_ADDRESS, I2C_READ);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	/* Cleaning ADDR condition sequence. */
	uint32_t reg32 = I2C_SR2(i2c);
	(void) reg32; /* unused */

	int i = 0;
	for (i = 0; i < n; ++i) {
		if(i == n - 1) {
			i2c_disable_ack(i2c);
		}
		while (!(I2C_SR1(i2c) & I2C_SR1_RxNE));
		res[i] = i2c_get_data(i2c);
	}
	i2c_send_stop(i2c);

	return;
}

static void codec_readid(void)
{
	sht21_send_cmd(I2C1, SHT21_CMD_READ_REG);
//	uint8_t raw = sht21_read1(I2C1);
	uint8_t raw;
	sht21_readn(I2C1, 1, &raw);
	printf("raw user reg = %#x\n", raw);
	int resolution = ((raw & 0x80) >> 6) | (raw & 1);
	printf("temp resolution is in %d bits\n", 14 - resolution);
	printf("battery status: %s\n", (raw & (1 << 6) ? "failing" : "good"));
	printf("On chip heater: %s\n", (raw & 0x2) ? "on" : "off");
	
	uint8_t req1[] = { 0xfa, 0x0f };
	uint8_t res[8];
	sht21_send_data(I2C1, 2, req1);
	sht21_readn(I2C1, sizeof(res), res);
	uint8_t req2[] = { 0xfc, 0xc9 };
	uint8_t res2[8];
	sht21_send_data(I2C1, 2, req2);
	sht21_readn(I2C1, sizeof(res), res2);
	printf("Serial = %02x%02x %02x%02x %02x%02x %02x%02x\n",
		res2[3], res2[4], res[0], res[2], res[4], res[6], res2[0], res2[1]);
}

static float sht21_convert_temp(uint16_t raw)
{
	//assert((raw & 0x2) == 0x2);
	raw &= ~0x3; /* Clear lower status bits */
	float tf = -46.85 + 175.72 * ((float) raw / 65536.0);
	return tf;
}

static float sht21_convert_humi(uint16_t raw)
{
	//assert((raw & 0x2) == 0);
	raw &= ~0x3; /* Clear lower status bits */
	float tf = -6 + 125 * ((float) raw / 65536.0);
	return tf;
}

static float sht21_read_temp_hold(uint32_t i2c)
{
	gpio_set(LED_DISCO_BLUE_PORT, LED_DISCO_BLUE_PIN);
	sht21_send_cmd(i2c, SHT21_CMD_TEMP_HOLD);

	//uint16_t left = sht21_read2(i2c);
	//uint16_t left = sht21_read2(i2c);
	uint8_t data[3];
	sht21_readn(i2c, 2, data);
	uint8_t crc = data[2];
	uint16_t temp = data[0] << 8 | data[1];
	printf("CRC=%#x, data0=%#x, data1=%#x\n", crc, data[0], data[1]);
	gpio_clear(LED_DISCO_BLUE_PORT, LED_DISCO_BLUE_PIN);
	return sht21_convert_temp(temp);
}

static float sht21_read_humi_hold(uint32_t i2c)
{
	gpio_set(LED_DISCO_BLUE_PORT, LED_DISCO_BLUE_PIN);
	sht21_send_cmd(i2c, SHT21_CMD_HUMIDITY_HOLD);

	uint16_t left = sht21_read2(i2c);
	gpio_clear(LED_DISCO_BLUE_PORT, LED_DISCO_BLUE_PIN);
	return sht21_convert_humi(left);
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

	while (1) {
		float temp = sht21_read_temp_hold(I2C1);
		float humi = sht21_read_humi_hold(I2C1);
		printf("Temp: %f C, RH: %f\n", temp, humi);
		for (j = 0; j < 100000; j++) { /* Wait a bit. */
			__asm__("NOP");
		}
		gpio_toggle(LED_DISCO_GREEN_PORT, LED_DISCO_GREEN_PIN);

	}
	return 0;
}
