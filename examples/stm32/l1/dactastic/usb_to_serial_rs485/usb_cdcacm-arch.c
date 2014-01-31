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
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include "usb_cdcacm.h"
#include "syscfg.h"
#include "rclog.h"

void usb_cdcacm_setup_pre_arch(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_DMA1);
}

void usb_cdcacm_setup_post_arch(void)
{
	/* Better enable interrupts */
	nvic_enable_irq(NVIC_USB_LP_IRQ);
	nvic_enable_irq(NVIC_CONF_DMA_USART);
}

volatile bool dma_write_in_progress = false;
volatile int dma_errors = 0;
static void dma_write(uint8_t *data, int size)
{	
        /* Reset DMA channel*/
        dma_channel_reset(DMA1, DMA_CHANNEL_USART_WRITE);

        dma_set_peripheral_address(DMA1, DMA_CHANNEL_USART_WRITE, (uint32_t)&USART_DR(USART_MODBUS));
        dma_set_memory_address(DMA1, DMA_CHANNEL_USART_WRITE, (uint32_t)data);
        dma_set_number_of_data(DMA1, DMA_CHANNEL_USART_WRITE, size);
        dma_set_read_from_memory(DMA1, DMA_CHANNEL_USART_WRITE);
        dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL_USART_WRITE);
        dma_set_peripheral_size(DMA1, DMA_CHANNEL_USART_WRITE, DMA_CCR_PSIZE_8BIT);
        dma_set_memory_size(DMA1, DMA_CHANNEL_USART_WRITE, DMA_CCR_MSIZE_8BIT);
        dma_set_priority(DMA1, DMA_CHANNEL_USART_WRITE, DMA_CCR_PL_VERY_HIGH);

        dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL_USART_WRITE);

        dma_enable_channel(DMA1, DMA_CHANNEL_USART_WRITE);

	// Make sure the tx complete is clear first
	USART_CR1(USART_MODBUS) &= ~USART_CR1_TCIE;
	USART_SR(USART_MODBUS) &= ~USART_SR_TC;
        usart_enable_tx_dma(USART_MODBUS);
	dma_write_in_progress = true;
	
}

void DMA_CHANNEL_USART_WRITE_IRQ_HANDLER(void)
{
	if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL_USART_WRITE, DMA_TCIF)) {
		// Enable transmit complete interrupt
		USART_CR1(USART_MODBUS) |= USART_CR1_TCIE;
		dma_clear_interrupt_flags(DMA1, DMA_CHANNEL_USART_WRITE, DMA_TCIF);
		dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL_USART_WRITE);
	        usart_disable_tx_dma(USART_MODBUS);
		dma_disable_channel(DMA1, DMA_CHANNEL_USART_WRITE);
//		dma_write_in_progress = false;
        }
#if 0
	if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL_USART_WRITE, DMA_TEIF)) {
		dma_clear_interrupt_flags(DMA1, DMA_CHANNEL_USART_WRITE, DMA_TEIF);
		dma_errors++;
		rclog_log("dma_errors: %d", dma_errors, 0);
	}
#endif
}

void glue_send_data_cb(uint8_t *buf, uint16_t len)
{
	gpio_set(LED_TX_PORT, LED_TX_PIN);
	gpio_set(RS485DE_PORT, RS485DE_PIN);
#define USE_DMA 1
#if USE_DMA
//	while (dma_write_in_progress) {
//		;
//	}
	dma_write(buf, len);
#else
	int i;
	//rclog_log("g_send_cb %d bytes: ", len, 0);
	for (i = 0; i < len; i++) {
		usart_send_blocking(USART_MODBUS, buf[i]);
	}
	gpio_clear(LED_TX_PORT, LED_TX_PIN);
	gpio_clear(RS485DE_PORT, RS485DE_PIN);
#endif
}

void glue_set_line_state_cb(uint8_t dtr, uint8_t rts)
{
	(void) dtr;
	(void) rts;
	// LM4f has an implementation of this if you're keen
}

int glue_set_line_coding_cb(uint32_t baud, uint8_t databits,
	enum usb_cdc_line_coding_bParityType cdc_parity,
	enum usb_cdc_line_coding_bCharFormat cdc_stopbits)
{
	int uart_parity;
	int uart_stopbits;

	if (databits < 8 || databits > 9) {
		return 0;
	}

	/* Be careful here, ST counts parity as a data bit */
	switch (cdc_parity) {
	case USB_CDC_NO_PARITY:
		uart_parity = USART_PARITY_NONE;
		break;
	case USB_CDC_ODD_PARITY:
		uart_parity = USART_PARITY_ODD;
		databits++;
		break;
	case USB_CDC_EVEN_PARITY:
		uart_parity = USART_PARITY_EVEN;
		databits++;
		break;
	default:
		return 0;
	}

	switch (cdc_stopbits) {
	case USB_CDC_1_STOP_BITS:
		uart_stopbits = USART_STOPBITS_1;
		break;
	case USB_CDC_2_STOP_BITS:
		uart_stopbits = USART_STOPBITS_2;
		break;
	default:
		return 0;
	}

	/* Disable the UART while we mess with its settings */
	usart_disable(USART_MODBUS);
	/* Set communication parameters */
	usart_set_baudrate(USART_MODBUS, baud);
	usart_set_databits(USART_MODBUS, databits);
	usart_set_parity(USART_MODBUS, uart_parity);
	usart_set_stopbits(USART_MODBUS, uart_stopbits);
	/* Back to work. */
	usart_enable(USART_MODBUS);

	return 1;
}
