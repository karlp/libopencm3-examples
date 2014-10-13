
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
/*
 * Basic interface to a Sensiron SHT21 temp/humidity sensor
 * Docs claim SCL can go up to 400kHz, so, that's fast mode, but this runs in
 * regular 100kHz mode 
 */

#include <stdint.h>
#include <libopencm3/stm32/i2c.h>
#include "sht21.h"

#define SHT21_ADDRESS	0x40
static uint32_t _i2c;

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

/**
 * Init, anything that needs doing....
 * 
 * @param i2c which i2c peripheral to use
 */
void sht21_begin(uint32_t i2c)
{
	_i2c = i2c;
}

static void sht21_send_cmd(uint32_t i2c, uint8_t cmd)
{
	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
#if 0
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
#else
	while(!(I2C_SR1(i2c) & I2C_SR1_SB));
#endif

	/* Send destination address. */
	i2c_send_7bit_address(i2c, SHT21_ADDRESS, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	uint32_t reg32 = I2C_SR2(i2c);
	(void) reg32; /* unused */

	/* Sending the data. */
	i2c_send_data(i2c, cmd);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

#if 0
	/* Send STOP condition. */
	i2c_send_stop(i2c);
#endif
}

/* 2 byte read is a "special case" thanks ST */
static int sht21_read2(uint32_t i2c, uint8_t *res) {
		/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Say to what address we want to talk to. */
	i2c_send_7bit_address(i2c, SHT21_ADDRESS, I2C_READ); 

	/* 2-byte receive is a special case. See datasheet POS bit. */
	I2C_CR1(i2c) |= (I2C_CR1_POS | I2C_CR1_ACK);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	uint32_t reg32 = I2C_SR2(i2c);
	(void)reg32;

	/* Cleaning I2C_SR1_ACK. */
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;

#if 0
	/* Now the slave should begin to send us the first byte. Await BTF. */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));

	/*
	 * Yes they mean it: we have to generate the STOP condition before
	 * saving the 1st byte.
	 */
	i2c_send_stop(i2c);

	res[0] = I2C_DR(i2c);
	res[1] = I2C_DR(i2c);

	/* Original state. */
	I2C_CR1(i2c) &= ~I2C_CR1_POS;
#endif
	/* Now the slave should begin to send us the first byte. Await BTF. */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	res[0] = (I2C_DR(i2c) << 8); /* MSB */

	/*
	 * Yes they mean it: we have to generate the STOP condition before
	 * saving the 1st byte.
	 */
	I2C_CR1(i2c) |= I2C_CR1_STOP;

	res[1] |= I2C_DR(i2c); /* LSB */

	/* Original state. */
	I2C_CR1(i2c) &= ~I2C_CR1_POS;

	return 0;
}

float sht21_read_temperature(void) {
	//sht21_send_cmd(_i2c, SHT21_CMD_TEMP_HOLD);
	sht21_send_cmd(_i2c, SHT21_CMD_TEMP_NOHOLD);
	uint8_t res[2];
	sht21_read2(_i2c, res);
	uint16_t temp = res[0] << 8 | res[1];
	//assert((temp & 0x2) == 0x2);
	temp &= ~0x3; /* Clear lower status bits */
	float tf = -46.85 + 175.72 * ((float)temp / 65536.0);
	return tf;
}
