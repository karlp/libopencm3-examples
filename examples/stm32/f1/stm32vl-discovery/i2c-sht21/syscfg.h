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
 * Things that are specific to this board, with different syscfg,
 * and a few makefile tweaks, this example should be usable on other boards.
 */
#ifndef SYSCFG_H
#define	SYSCFG_H
BEGIN_DECLS
        
#include <libopencm3/stm32/rcc.h>

/* F1 version */
#define CONSOLE_UART    USART2
#define CONSOLE_UART_RCC RCC_USART2
#define CONSOLE_UART_RCC_GPIO RCC_GPIOA
#define CONSOLE_UART_GPIO_PORT GPIOA
#define CONSOLE_UART_GPIO_PINS GPIO_USART2_TX

/* DISCO LEDS VL */
#define LED_DISCO_RCC   RCC_GPIOC
#define LED_DISCO_BLUE_PORT GPIOC
#define LED_DISCO_BLUE_PIN GPIO8
#define LED_DISCO_GREEN_PORT GPIOC
#define LED_DISCO_GREEN_PIN GPIO9

#define I2C_SENSOR  I2C1
#define I2C_SENSOR_RCC RCC_I2C1
#define I2C_SENSOR_RCC_GPIO RCC_GPIOB
#define I2C_SENSOR_GPIO_PORT GPIOB
#define I2C_SENSOR_GPIO_PINS (GPIO_I2C1_SCL | GPIO_I2C1_SDA)

END_DECLS
#endif	/* SYSCFG_H */
