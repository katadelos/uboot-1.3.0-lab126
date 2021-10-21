/*
 * Copyright (C) 2007, Lab126, Inc.
 */

#include <asm/arch/iomux.h>


void wan_hw_init(void)
{
	mxc_request_iomux(MX31_PIN_UART1_RTS, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);     // WAN_ON_OFF
	mxc_set_gpio_direction(MX31_PIN_UART1_RTS, 0);
	mxc_set_gpio_dataout(MX31_PIN_UART1_RTS, 0);

	mxc_request_iomux(MX31_PIN_CSI_PIXCLK, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);    // MODULE_WAKE
	mxc_set_gpio_direction(MX31_PIN_CSI_PIXCLK, 0);
	mxc_set_gpio_dataout(MX31_PIN_CSI_PIXCLK, 0);

	mxc_request_iomux(MX31_PIN_USB_PWR, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);       // USBHC1_PWR
	mxc_set_gpio_direction(MX31_PIN_USB_PWR, 0);
	mxc_set_gpio_dataout(MX31_PIN_USB_PWR, 0);
}

