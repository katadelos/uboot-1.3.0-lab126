/*
 * (C) Copyright 2003
 * Gerry Hamel, geh@ti.com, Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 *
 */

#ifndef __USB_TTY_H__
#define __USB_TTY_H__


#include "usbdcore.h"

#define NUM_CONFIGS    1
#define NUM_INTERFACES 1
#define NUM_ENDPOINTS  2

#define EP0_MAX_PACKET_SIZE 16

#define CONFIG_USBD_CONFIGURATION_STR "BootUSB"
#define CONFIG_USBD_INTERFACE_STR     "boot_usb interface"


#define CONFIG_USBD_BOOTUSB_OUT_ENDPOINT 2
#define CONFIG_USBD_BOOTUSB_OUT_PKTSIZE	512 /* 64 */
#define CONFIG_USBD_BOOTUSB_IN_ENDPOINT	1
#define CONFIG_USBD_BOOTUSB_IN_PKTSIZE	512 /* 64 */

#define BOOTUSB_DEVICE_CLASS	0xAA
#define BOOTUSB_DEVICE_SUBCLASS	0x00
#define BOOTUSB_DEVICE_PROTOCOL	0x00

#define BOOTUSB_INTERFACE_CLASS	   0xFF /* Vendor Specific */
#define BOOTUSB_INTERFACE_SUBCLASS  0x02
#define BOOTUSB_INTERFACE_PROTOCOL  0x01

#define BOOTUSB_BCD_DEVICE 0x0
#define BOOTUSB_MAXPOWER  0x0

#define STR_MANUFACTURER 1
#define STR_PRODUCT	 2
#define STR_SERIAL	 3
#define STR_CONFIG	 4
#define STR_INTERFACE	 5

#define NUL     0x00
#define STX     0x02
#define ETX     0x03
#define RS      0x1E

#endif
