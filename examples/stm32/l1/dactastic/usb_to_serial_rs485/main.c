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

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>

#include "syscfg.h"
#include "usb_cdcacm.h"

static void usart_setup(void)
{
	/* Enable the USART interrupt. */
	nvic_enable_irq(NVIC_CONF_USART);

	/* USART pins */
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(USART_CONF_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_CONF_PINS);
	gpio_set_af(USART_CONF_PORT, GPIO_AF7, USART_CONF_PINS);

	rcc_periph_clock_enable(RCC_CONF_USART);

	/* Setup USART parameters. */
	usart_set_baudrate(USART_MODBUS, 115200);
	usart_set_databits(USART_MODBUS, 8);
	usart_set_stopbits(USART_MODBUS, USART_STOPBITS_1);
	usart_set_mode(USART_MODBUS, USART_MODE_TX_RX);
	usart_set_parity(USART_MODBUS, USART_PARITY_NONE);
	usart_set_flow_control(USART_MODBUS, USART_FLOWCONTROL_NONE);

	/* Enable USART_MODBUS Receive interrupt. */
	usart_enable_rx_interrupt(USART_MODBUS);

	/* Finally enable the USART. */
	usart_enable(USART_MODBUS);
}

/*void usb_lp_isr(void)
{
	usbd_poll(usbd_dev);
}
 * */

void USART_CONF_ISR(void)
{
	/* Check if we were called because of RXNE. */
	if ((USART_CR1(USART_MODBUS) & USART_CR1_RXNEIE) &&
		(USART_SR(USART_MODBUS) & USART_SR_RXNE)) {
		gpio_set(LED_RX_PORT, LED_RX_PIN);
		uint8_t c = usart_recv(USART_MODBUS);
		glue_data_received_cb(&c, 1);
		gpio_clear(LED_RX_PORT, LED_RX_PIN);
	}
	if ((USART_CR1(USART_MODBUS) & USART_CR1_TCIE) &&
		(USART_SR(USART_MODBUS) & USART_SR_TC)) {
		USART_CR1(USART_MODBUS) &= ~USART_CR1_TCIE;
		USART_SR(USART_MODBUS) &= ~USART_SR_TC;
		gpio_clear(RS485DE_PORT, RS485DE_PIN);
		gpio_clear(LED_TX_PORT, LED_TX_PIN);
	}
}

const clock_scale_t this_clock_config =
        { /* 32MHz PLL from 8MHz HSE */
                .pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
                .pll_mul = RCC_CFGR_PLLMUL_MUL12,
                .pll_div = RCC_CFGR_PLLDIV_DIV3,
                .hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
                .ppre1 = RCC_CFGR_PPRE1_HCLK_NODIV,
                .ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,
                .voltage_scale = RANGE1,
                .flash_config = FLASH_ACR_LATENCY_1WS,
                .apb1_frequency = 32000000,
                .apb2_frequency = 32000000,
        };


int main(void)
{
	usbd_device *usbd_dev;

	//rcc_clock_setup_pll(&clock_config[CLOCK_VRANGE1_HSI_PLL_32MHZ]);
	rcc_clock_setup_pll_osc(HSE, &this_clock_config);

	/* Leds and rs485 are on port b */
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(LED_RX_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		LED_RX_PIN | LED_TX_PIN);
	gpio_mode_setup(RS485DE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		RS485DE_PIN);
	gpio_set(LED_RX_PORT, LED_RX_PIN);

	//usart_setup();

	usb_cdcacm_init(&usbd_dev);

	while (1) {
		usbd_poll(usbd_dev);
	}

}

