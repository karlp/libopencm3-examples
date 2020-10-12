/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
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

#include <string.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dfu.h>

#define APP_ADDRESS	0x08002000

/* explicit actions requested */
#define BF_BOOT_USER 0xb0079001
#define BF_BOOT_BL 0x55aab007
static volatile uint32_t boot_flag __attribute__((section(".noinit")));

/* Commands sent with wBlockNum == 0 as per ST implementation. */
#define CMD_SETADDR	0x21
#define CMD_ERASE	0x41

/* We need a special large control buffer for this device: */
uint8_t usbd_control_buffer[1024];


#include <libopencm3/cm3/itm.h>
static void trace_send_blocking8(int stimulus_port, char c) {
	if (!(ITM_TER[0] & (1<<stimulus_port))) {
		return;
	}
	while (!(ITM_STIM8(stimulus_port) & ITM_STIM_FIFOREADY))
		;
	ITM_STIM8(stimulus_port) = c;
}


static enum dfu_state usbdfu_state = STATE_DFU_IDLE;

static struct {
	uint8_t buf[sizeof(usbd_control_buffer)];
	uint16_t len;
	uint32_t addr;
	uint16_t blocknum;
} prog;

const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0xDF11,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_UPLOAD | USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = 128, /* half page on L1 only please */
	/* this means you must use dfuse! */
	.bcdDFUVersion = 0x011A,
	/* this means normal dfu files */
	//.bcdDFUVersion = 0x0100,
};

const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE, /* Device Firmware Upgrade */
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 2,

	/* The ST Microelectronics DfuSe application needs this string.
	 * The format isn't documented... */
	.iInterface = 4,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Black Sphere Technologies",
	"DFU Demo",
	"DEMO",
	/* This string is used by ST Microelectronics' DfuSe utility. */
	"@Internal Flash   /0x08000000/32*256Ba,224*256Bg",
};

static uint8_t usbdfu_getstatus(uint32_t *bwPollTimeout)
{
	trace_send_blocking8(28, usbdfu_state);
	switch (usbdfu_state) {
	case STATE_DFU_DNLOAD_SYNC:
		trace_send_blocking8(29, 41);
		usbdfu_state = STATE_DFU_DNBUSY;
		/* we do half page writes at a time, so 5ms is more than enough to ask them to wait */
		*bwPollTimeout = 5;
		return DFU_STATUS_OK;
	case STATE_DFU_MANIFEST_SYNC:
		trace_send_blocking8(29, 42);
		/* Device will reset when read is complete. */
		usbdfu_state = STATE_DFU_MANIFEST;
		return DFU_STATUS_OK;
	default:
		return DFU_STATUS_OK;
	}
}

static void usbdfu_getstatus_complete(usbd_device *usbd_dev, struct usb_setup_data *req)
{
	(void)req;
	(void)usbd_dev;

	trace_send_blocking8(27, usbdfu_state);
	switch (usbdfu_state) {
	case STATE_DFU_DNBUSY:
		flash_unlock();
		if (prog.blocknum == 0) {
			switch (prog.buf[0]) {
			case CMD_ERASE:
				{
					uint32_t *dat = (uint32_t *)(prog.buf + 1);
					flash_erase_page(*dat);
				}
				break;
		
			case CMD_SETADDR:
				{
					uint32_t *dat = (uint32_t *)(prog.buf + 1);
					prog.addr = *dat;
				}
				break;
			}
		} else {
			uint32_t baseaddr = prog.addr + ((prog.blocknum - 2) *
				       dfu_function.wTransferSize);
		        cm_disable_interrupts();
			/* this is to autoerase as we go, regardless of ST DfuSe extensions (CMD_ERASE) */
#if 0
			if ((baseaddr & (256 - 1)) == 0) {
				flash_erase_page(baseaddr);
			}
#endif
			flash_program_half_page((uint32_t *)baseaddr, prog.buf);
			cm_enable_interrupts();
		}
		flash_lock();

		/* Jump straight to dfuDNLOAD-IDLE, skipping dfuDNLOAD-SYNC. */
		usbdfu_state = STATE_DFU_DNLOAD_IDLE;
		return;
	case STATE_DFU_MANIFEST:
		/* USB device must detach, we just reset... */
		trace_send_blocking8(29, 43);
		for (int i = 0; i < 10000; i++) {
			__asm__("nop");
		}
		scb_reset_system();
		return; /* Will never return. */
	default:
		trace_send_blocking8(29, 46);
		return;
	}
}

static enum usbd_request_return_codes usbdfu_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)usbd_dev;

	if ((req->bmRequestType & 0x7F) != 0x21)
		return USBD_REQ_NOTSUPP; /* Only accept class request. */

	switch (req->bRequest) {
	case DFU_DNLOAD:
		if ((len == NULL) || (*len == 0)) {
			trace_send_blocking8(29, 47);
			usbdfu_state = STATE_DFU_MANIFEST_SYNC;
		} else {
			/* Copy download data for use on GET_STATUS. */
			prog.blocknum = req->wValue;
			prog.len = *len;
			memcpy(prog.buf, *buf, *len);
			usbdfu_state = STATE_DFU_DNLOAD_SYNC;
		}
		return USBD_REQ_HANDLED;
	case DFU_CLRSTATUS:
		/* Clear error and return to dfuIDLE. */
		if (usbdfu_state == STATE_DFU_ERROR)
			usbdfu_state = STATE_DFU_IDLE;
		return USBD_REQ_HANDLED;
	case DFU_ABORT:
		trace_send_blocking8(29, 10);
		/* Abort returns to dfuIDLE state. */
		usbdfu_state = STATE_DFU_IDLE;
		return USBD_REQ_HANDLED;
	case DFU_UPLOAD:
		usbdfu_state = STATE_DFU_UPLOAD_IDLE;
		uint32_t addr = prog.addr + ((req->wValue - 2) * dfu_function.wTransferSize);
		*buf = (void*)addr;
		*len = dfu_function.wTransferSize;
		return USBD_REQ_HANDLED;
	case DFU_GETSTATUS: {
		uint32_t bwPollTimeout = 0; /* 24-bit integer in DFU class spec */
		//trace_send_blocking8(29, 40);
		(*buf)[0] = usbdfu_getstatus(&bwPollTimeout);
		(*buf)[1] = bwPollTimeout & 0xFF;
		(*buf)[2] = (bwPollTimeout >> 8) & 0xFF;
		(*buf)[3] = (bwPollTimeout >> 16) & 0xFF;
		(*buf)[4] = usbdfu_state;
		(*buf)[5] = 0; /* iString not used here */
		*len = 6;
		*complete = usbdfu_getstatus_complete;
		return USBD_REQ_HANDLED;
		}
	case DFU_GETSTATE:
		/* Return state with no state transision. */
		trace_send_blocking8(29, 50);
		*buf[0] = usbdfu_state;
		*len = 1;
		return USBD_REQ_HANDLED;
	case DFU_DETACH:
		/* we're being asked to reboot into user code */
		boot_flag = BF_BOOT_USER;
		scb_reset_system();
		return USBD_REQ_HANDLED;
	default:
		trace_send_blocking8(29, 60);
		trace_send_blocking8(26, req->bRequest);
	}

	return USBD_REQ_NOTSUPP;
}

static void usbdfu_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usbdfu_control_request);
}

int main(void)
{
	/* If requested, boot _immediately_ to user code.
	 * we do this immediately to ensure user code has the cleanest
	 * possible environment
	 */
	trace_send_blocking8(29, 1);
	if (boot_flag == BF_BOOT_USER) {
		trace_send_blocking8(29, 2);
		boot_flag = 0;
		/* Boot the application if it's valid. */
		if ((*(volatile uint32_t *)APP_ADDRESS & 0x2FFE0000) == 0x20000000) {
			trace_send_blocking8(29, 3);
			/* Set vector table base address. */
			SCB_VTOR = APP_ADDRESS & 0xFFFF;
			/* Initialise master stack pointer. */
			__asm__ volatile("msr msp, %0"::"g"
				     (*(volatile uint32_t *)APP_ADDRESS));
			/* Jump to application. */
			(*(void (**)())(APP_ADDRESS + 4))();
		}
	}

	/*
	 * ok, start the bootloader instead, but if we reset again, attempt
	 * to start user code on next boot again
	 */
	boot_flag = BF_BOOT_USER;

	usbd_device *usbd_dev;
	/* turn off the usb pull up right now, helps us re-enumerate */
        SYSCFG_PMC &= ~SYSCFG_PMC_USB_PU;

	/* any L1 with a 16MHz crystal */
	const struct rcc_clock_scale myclock_16mhz_hse = {
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.pll_mul = RCC_CFGR_PLLMUL_MUL6,
		.pll_div = RCC_CFGR_PLLDIV_DIV3,
		.hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
		.ppre1 = RCC_CFGR_PPRE1_HCLK_NODIV,
		.ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,
		.voltage_scale = PWR_SCALE1,
		.flash_waitstates = 1,
		.ahb_frequency = 32e6,
		.apb1_frequency = 32e6,
		.apb2_frequency = 32e6,
	};
	rcc_clock_setup_pll(&myclock_16mhz_hse);

	trace_send_blocking8(29, 4);
	/* Enable built in USB pullup on L1 */
        rcc_periph_clock_enable(RCC_SYSCFG);
        SYSCFG_PMC |= SYSCFG_PMC_USB_PU;

	/*
	 * TODO: optionally, configure and read GPIOs here for button control.
	 * set boot_flag and scb_reset_system() as desired.
	 */

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 4, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, usbdfu_set_config);

	trace_send_blocking8(29, 5);
	while (1)
		usbd_poll(usbd_dev);
}