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

#include <common.h>

#ifdef CONFIG_USB_BOOTUSB

#include <malloc.h>
#include <circbuf.h>
#include <devices.h>
#include <asm/byteorder.h>
#include "usb_bootusb.h"
#include "udc.h"

/*
 * Instance variables
 */
static struct usb_device_instance	 device_instance[1];
static struct usb_bus_instance		 bus_instance[1];
static struct usb_configuration_instance config_instance[NUM_CONFIGS];
static struct usb_interface_instance	 interface_instance[NUM_INTERFACES];
static struct usb_alternate_instance	 alternate_instance[NUM_INTERFACES];
static struct usb_endpoint_instance	 endpoint_instance[NUM_ENDPOINTS+1]; /* one extra for control endpoint */

/*
 * Static allocation of urbs
 */
#define RECV_ENDPOINT	1
#define TX_ENDPOINT	2

/*
 * Global flag
 */
int bootusb_configured_flag = 0;


/*
 * Serial number
 */
static char serial_number[16];

/*
 * Descriptors
 */
static u8 wstrLang[4] = {4,USB_DT_STRING,0x9,0x4};
static u8 wstrManufacturer[2 + 2*(sizeof(CONFIG_USBD_MANUFACTURER)-1)];
static u8 wstrProduct[2 + 2*(sizeof(CONFIG_USBD_PRODUCT_NAME)-1)];
static u8 wstrSerial[2 + 2*(sizeof(serial_number) - 1)];
static u8 wstrConfiguration[2 + 2*(sizeof(CONFIG_USBD_CONFIGURATION_STR)-1)];
static u8 wstrInterface[2 + 2*(sizeof(CONFIG_USBD_INTERFACE_STR)-1)];

static struct usb_string_descriptor *bootusb_string_table[] = {
  (struct usb_string_descriptor*)wstrLang,
  (struct usb_string_descriptor*)wstrManufacturer,
  (struct usb_string_descriptor*)wstrProduct,
  (struct usb_string_descriptor*)wstrSerial,
  (struct usb_string_descriptor*)wstrConfiguration,
  (struct usb_string_descriptor*)wstrInterface
};
extern struct usb_string_descriptor **usb_strings; /* defined and used by omap1510_ep0.c */

static struct usb_device_descriptor device_descriptor = {
  bLength:	      sizeof(struct usb_device_descriptor),
  bDescriptorType:    USB_DT_DEVICE,
  bcdUSB:	      USB_BCD_VERSION,
  bDeviceClass:	      BOOTUSB_DEVICE_CLASS,
  bDeviceSubClass:    BOOTUSB_DEVICE_SUBCLASS,
  bDeviceProtocol:    BOOTUSB_DEVICE_PROTOCOL,
  bMaxPacketSize0:    EP0_MAX_PACKET_SIZE,
  idVendor:	      CONFIG_USBD_VENDORID,
  idProduct:	      CONFIG_USBD_PRODUCTID,
  bcdDevice:	      BOOTUSB_BCD_DEVICE,
  iManufacturer:      STR_MANUFACTURER,
  iProduct:	      STR_PRODUCT,
  iSerialNumber:      STR_SERIAL,
  bNumConfigurations: NUM_CONFIGS
  };
static struct usb_configuration_descriptor config_descriptors[NUM_CONFIGS] = {
  {
    bLength:		 sizeof(struct usb_configuration_descriptor),
    bDescriptorType:	 USB_DT_CONFIG,
    wTotalLength:	 (sizeof(struct usb_configuration_descriptor)*NUM_CONFIGS) +
			 (sizeof(struct usb_interface_descriptor)*NUM_INTERFACES) +
			 (sizeof(struct usb_endpoint_descriptor)*NUM_ENDPOINTS),
    bNumInterfaces:	 NUM_INTERFACES,
    bConfigurationValue: 1,
    iConfiguration:	 STR_CONFIG,
    bmAttributes:	 BMATTRIBUTE_SELF_POWERED | BMATTRIBUTE_RESERVED,
    bMaxPower:		 BOOTUSB_MAXPOWER
  },
};
static struct usb_interface_descriptor interface_descriptors[NUM_INTERFACES] = {
  {
    bLength:		 sizeof(struct usb_interface_descriptor),
    bDescriptorType:	 USB_DT_INTERFACE,
    bInterfaceNumber:	 0,
    bAlternateSetting:	 0,
    bNumEndpoints:	 NUM_ENDPOINTS,
    bInterfaceClass:	 BOOTUSB_INTERFACE_CLASS,
    bInterfaceSubClass:	 BOOTUSB_INTERFACE_SUBCLASS,
    bInterfaceProtocol:	 BOOTUSB_INTERFACE_PROTOCOL,
    iInterface:		 STR_INTERFACE
  },
};
static struct usb_endpoint_descriptor ep_descriptors[NUM_ENDPOINTS] = {
  {
    bLength:		 sizeof(struct usb_endpoint_descriptor),
    bDescriptorType:	 USB_DT_ENDPOINT,
    bEndpointAddress:	 CONFIG_USBD_BOOTUSB_OUT_ENDPOINT | USB_DIR_OUT,
    bmAttributes:	 USB_ENDPOINT_XFER_BULK,
    wMaxPacketSize:	 CONFIG_USBD_BOOTUSB_OUT_PKTSIZE ,
    bInterval:		 0
  },
  {
    bLength:		 sizeof(struct usb_endpoint_descriptor),
    bDescriptorType:	 USB_DT_ENDPOINT,
    bEndpointAddress:	 CONFIG_USBD_BOOTUSB_IN_ENDPOINT | USB_DIR_IN,
    bmAttributes:	 USB_ENDPOINT_XFER_BULK,
    wMaxPacketSize:	 CONFIG_USBD_BOOTUSB_IN_PKTSIZE,
    bInterval:		 0
  },
};
static struct usb_endpoint_descriptor *ep_descriptor_ptrs[NUM_ENDPOINTS] = {
  &(ep_descriptors[0]),
  &(ep_descriptors[1]),
};

/* utility function for converting char* to wide string used by USB */
static void str2wide (char *str, u16 * wide)
{
	int i;

	for (i = 0; i < strlen (str) && str[i]; i++)
		wide[i] = (u16) str[i];
}

/*
 * Prototypes
 */
static void bootusb_init_strings (void);
static void bootusb_init_instances (void);
static void bootusb_init_endpoints (void);

static void bootusb_event_handler (struct usb_device_instance *device,
				  usb_device_event_t event, int data);
static int bootusb_configured (void);
void bootusb_poll (void);

circbuf_t usb_input;

/*
 * Initialize the usb client port.
 *
 */
int drv_bootusb_init (void)
{
	char * sn;
	int snlen;

	if (!(sn = getenv("serial#"))) {
		sn = "000000000000";
	}
	snlen = strlen(sn);
	if (snlen > sizeof(serial_number) - 1) {
		printf ("Warning: serial number %s is too long (%d > %d)\n",
			sn, snlen, sizeof(serial_number) - 1);
		snlen = sizeof(serial_number) - 1;
	}
	memcpy (serial_number, sn, snlen);
	serial_number[snlen] = '\0';

	buf_init(&usb_input, 0x4000);

	printf("bootusb: ready to accept connection\n");

	/* Now, set up USB controller and infrastructure */
	udc_init ();		/* Basic USB initialization */

	bootusb_init_strings ();
	bootusb_init_instances ();

	udc_startup_events (device_instance);	/* Enable our device, initialize udc pointers */
	udc_connect ();		/* Enable pullup for host detection */

	bootusb_init_endpoints ();

	return 0;
}

static void bootusb_init_strings (void)
{
	struct usb_string_descriptor *string;

	string = (struct usb_string_descriptor *) wstrManufacturer;
	string->bLength = sizeof (wstrManufacturer);
	string->bDescriptorType = USB_DT_STRING;
	str2wide (CONFIG_USBD_MANUFACTURER, string->wData);

	string = (struct usb_string_descriptor *) wstrProduct;
	string->bLength = sizeof (wstrProduct);
	string->bDescriptorType = USB_DT_STRING;
	str2wide (CONFIG_USBD_PRODUCT_NAME, string->wData);

	string = (struct usb_string_descriptor *) wstrSerial;
	string->bLength = 2 + 2*strlen(serial_number);
	string->bDescriptorType = USB_DT_STRING;
	str2wide (serial_number, string->wData);

	string = (struct usb_string_descriptor *) wstrConfiguration;
	string->bLength = sizeof (wstrConfiguration);
	string->bDescriptorType = USB_DT_STRING;
	str2wide (CONFIG_USBD_CONFIGURATION_STR, string->wData);

	string = (struct usb_string_descriptor *) wstrInterface;
	string->bLength = sizeof (wstrInterface);
	string->bDescriptorType = USB_DT_STRING;
	str2wide (CONFIG_USBD_INTERFACE_STR, string->wData);

	/* Now, initialize the string table for ep0 handling */
	usb_strings = bootusb_string_table;
}

static void bootusb_init_instances (void)
{
	int i;

	/* initialize device instance */
	memset (device_instance, 0, sizeof (struct usb_device_instance));
	device_instance->device_state = STATE_INIT;
	device_instance->device_descriptor = &device_descriptor;
	device_instance->event = bootusb_event_handler;
	device_instance->bus = bus_instance;
	device_instance->configurations = NUM_CONFIGS;
	device_instance->configuration_instance_array = config_instance;

	/* initialize bus instance */
	memset (bus_instance, 0, sizeof (struct usb_bus_instance));
	bus_instance->device = device_instance;
	bus_instance->endpoint_array = endpoint_instance;
	bus_instance->max_endpoints = 1;
	bus_instance->maxpacketsize = 64;
	bus_instance->serial_number_str = serial_number;

	/* configuration instance */
	memset (config_instance, 0,
		sizeof (struct usb_configuration_instance));
	config_instance->interfaces = NUM_INTERFACES;
	config_instance->configuration_descriptor = config_descriptors;
	config_instance->interface_instance_array = interface_instance;

	/* interface instance */
	memset (interface_instance, 0,
		sizeof (struct usb_interface_instance));
	interface_instance->alternates = 1;
	interface_instance->alternates_instance_array = alternate_instance;

	/* alternates instance */
	memset (alternate_instance, 0,
		sizeof (struct usb_alternate_instance));
	alternate_instance->interface_descriptor = interface_descriptors;
	alternate_instance->endpoints = NUM_ENDPOINTS;
	alternate_instance->endpoints_descriptor_array = ep_descriptor_ptrs;

	/* endpoint instances */
	memset (&endpoint_instance[0], 0,
		sizeof (struct usb_endpoint_instance));
	endpoint_instance[0].endpoint_address = 0;
	endpoint_instance[0].rcv_packetSize = EP0_MAX_PACKET_SIZE;
	endpoint_instance[0].rcv_attributes = USB_ENDPOINT_XFER_CONTROL;
	endpoint_instance[0].tx_packetSize = EP0_MAX_PACKET_SIZE;
	endpoint_instance[0].tx_attributes = USB_ENDPOINT_XFER_CONTROL;
	udc_setup_ep (device_instance, 0, &endpoint_instance[0]);

	for (i = 1; i <= NUM_ENDPOINTS; i++) {
		memset (&endpoint_instance[i], 0,
			sizeof (struct usb_endpoint_instance));

		endpoint_instance[i].endpoint_address =
			ep_descriptors[i - 1].bEndpointAddress;

		endpoint_instance[i].rcv_packetSize =
			ep_descriptors[i - 1].wMaxPacketSize;
		endpoint_instance[i].rcv_attributes =
			ep_descriptors[i - 1].bmAttributes;

		endpoint_instance[i].tx_packetSize =
			ep_descriptors[i - 1].wMaxPacketSize;
		endpoint_instance[i].tx_attributes =
			ep_descriptors[i - 1].bmAttributes;

		urb_link_init (&endpoint_instance[i].rcv);
		urb_link_init (&endpoint_instance[i].rdy);
		urb_link_init (&endpoint_instance[i].tx);
		urb_link_init (&endpoint_instance[i].done);

		if (endpoint_instance[i].endpoint_address & USB_DIR_IN) {
			endpoint_instance[i].tx_urb =
				usbd_alloc_urb (device_instance,
						&endpoint_instance[i]);
		}
		else {
			endpoint_instance[i].rcv_urb =
				usbd_alloc_urb (device_instance,
						&endpoint_instance[i]);
		}
	}
}

static void bootusb_init_endpoints (void)
{
	int i;

	bus_instance->max_endpoints = NUM_ENDPOINTS + 1;
	for (i = 0; i <= NUM_ENDPOINTS; i++) {
		udc_setup_ep (device_instance, i, &endpoint_instance[i]);
	}
}

static void bootusb_event_handler (struct usb_device_instance *device,
				  usb_device_event_t event, int data)
{
	switch (event) {
	case DEVICE_RESET:
	case DEVICE_BUS_INACTIVE:
		bootusb_configured_flag = 0;
		break;
	case DEVICE_CONFIGURED:
		bootusb_configured_flag = 1;
		break;

	case DEVICE_ADDRESS_ASSIGNED:
		bootusb_init_endpoints ();

	default:
		break;
	}
}


#if 0
/*********************************************************************************/

static struct urb *next_urb (struct usb_device_instance *device,
			     struct usb_endpoint_instance *endpoint)
{
	struct urb *current_urb = NULL;
	int space;

	/* If there's a queue, then we should add to the last urb */
	if (!endpoint->tx_queue) {
		current_urb = endpoint->tx_urb;
	} else {
		/* Last urb from tx chain */
		current_urb =
			p2surround (struct urb, link, endpoint->tx.prev);
	}

	/* Make sure this one has enough room */
	space = current_urb->buffer_length - current_urb->actual_length;
	if (space > 0) {
		return current_urb;
	} else {		/* No space here */
		/* First look at done list */
		current_urb = first_urb_detached (&endpoint->done);
		if (!current_urb) {
			current_urb = usbd_alloc_urb (device, endpoint);
		}

		urb_append (&endpoint->tx, current_urb);
		endpoint->tx_queue++;
	}
	return current_urb;
}

static int write_buffer (circbuf_t * buf)
{
	if (!usbtty_configured ()) {
		return 0;
	}

	if (buf->size) {

		struct usb_endpoint_instance *endpoint =
			&endpoint_instance[TX_ENDPOINT];
		struct urb *current_urb = NULL;
		char *dest;

		int space_avail;
		int popnum, popped;
		int total = 0;

		/* Break buffer into urb sized pieces, and link each to the endpoint */
		while (buf->size > 0) {
			current_urb = next_urb (device_instance, endpoint);
			if (!current_urb) {
				TTYERR ("current_urb is NULL, buf->size %d\n",
					buf->size);
				return total;
			}

			dest = current_urb->buffer +
				current_urb->actual_length;

			space_avail =
				current_urb->buffer_length -
				current_urb->actual_length;
			popnum = MIN (space_avail, buf->size);
			if (popnum == 0)
				break;

			popped = buf_pop (buf, dest, popnum);
			if (popped == 0)
				break;
			current_urb->actual_length += popped;
			total += popped;

			/* If endpoint->last == 0, then transfers have not started on this endpoint */
			if (endpoint->last == 0) {
				udc_endpoint_write (endpoint);
			}

		}		/* end while */
		return total;
	}			/* end if tx_urb */

	return 0;
}
#endif

static int fill_buffer (circbuf_t * buf)
{
	struct usb_endpoint_instance *endpoint =
		&endpoint_instance[RECV_ENDPOINT];

	if (endpoint->rcv_urb && endpoint->rcv_urb->actual_length) {
		unsigned int nb = endpoint->rcv_urb->actual_length;
		char *src = (char *) endpoint->rcv_urb->buffer;

		buf_push (buf, src, nb);
		endpoint->rcv_urb->actual_length = 0;

		return nb;
	}

	return 0;
}

static int bootusb_configured (void)
{
	return bootusb_configured_flag;
}

void bootusb_confirm(void)
{
	struct usb_endpoint_instance *ep = &endpoint_instance[TX_ENDPOINT];
	int size = 1;
	u8 reply[size];

	reply[0] = 0xAC;
	ep->tx_urb->actual_length = 0;
	memcpy(ep->tx_urb->buffer +
                               ep->tx_urb->actual_length, reply , size);
	ep->tx_urb->actual_length += size;

	udc_endpoint_write(ep);
}

int bootusb_get(char *string, u8 **payload)
{
	char *cmdptr, command[12];
	char r;
	int bin = 0;
	int size;

	do {
		bootusb_poll();
	} while (usb_input.size <= 0);

#ifdef DEBUG
	printf("Got %d bytes\n", usb_input.size);
#endif
	cmdptr = NULL;
	while (usb_input.size) {
		buf_pop(&usb_input, &r, 1);
		if (!bin) {
			if (r == STX) {
				cmdptr = command;
			}
			else if (r == RS) {
				*cmdptr = '\0';
				cmdptr = 0;
				strcpy(string, command);
				bin = 1;
			}
			else if (r == ETX) {
				*cmdptr = '\0';
				cmdptr = 0;
				strcpy(string, command);
				return 0;
			}
			else
			{
				*cmdptr = r;
				cmdptr ++;
			}
		}
		else
		{
			size = usb_input.size;
			*payload = malloc(size + 1);
			(*payload)[0] = r;
			buf_pop(&usb_input, (*payload) + 1, size);
			if ((*payload)[size] != ETX) {
				printf("E R R O R! ETX-less packet received!\n");
			}
			return size;
		}
	}
	return 0;
}

static u8 ezx_csum(u8 *data, int len)
{
        u8 ret = 0;
        int i;

        for (i = 0; i < len; i++)
                ret += data[i];

        return ret;
}

void bootusb_loop(void)
{
	int size;
	u8 *p;
	char x_addr[11]; /* "0x12345678\0" */
	char x_csum[5];
	unsigned addr, csum, chunk_size;
	char cmd[20];
	int count = 0;
	u32 img_start = 0, img_size, img_tail;
	char flash_script[256];

	addr = 0x0; /* dummy value, will be overwritten */

	bootusb_get(cmd, &p);
	if (!strcmp(cmd, "RQSN"))
		bootusb_confirm();

	bootusb_get(cmd, &p);
	if (!strcmp(cmd, "RQVN"))
		bootusb_confirm();

	printf("Loading...\n\t");
	img_size = 0;

	do {
		size = bootusb_get(cmd, &p);
#ifdef DEBUG
		printf("CMD=%s, PAYLOAD=%d\n", cmd, size);
#endif
		if (!strcmp(cmd, "ADDR")) {
			strncpy(x_addr, (char*)p, 8);
			x_addr[8] = 0;

			memset(x_csum, 0, sizeof(x_csum));
			strncpy(x_csum, (char*)p + 8, 2);

			addr = simple_strtoul(x_addr, NULL, 16);
			csum = simple_strtoul(x_csum, NULL, 16);
#ifdef DEBUG
			printf("\n** bootusb: addr = %08x; csum = %08x\n", addr, csum);
#endif
		}
		else if (!strcmp(cmd, "JUMP")) {
			extern ulong load_addr; /* FIXME! trick to pass loadaddr to bootm... */

			memset(x_addr, 0, sizeof(x_addr));
			strncpy(x_addr, (char*)p, 8);
			addr =  simple_strtoul(x_addr, NULL, 16);
#ifdef DEBUG
			printf("\n** bootusb: JUMP to %x\n", addr);
#else
			printf("\n");
#endif
			bootusb_confirm();
			udelay(1000);
			udc_stop();

			load_addr = addr;
			do_bootm (NULL, 1, "bootm");
			/* nobody seems to get here... */
			for(;;);
		}
		else if (!strcmp(cmd, "FSRC")) {
			memset(x_addr, 0, sizeof(x_addr));
			strncpy(x_addr, (char*)p, 8);
			img_start =  simple_strtoul(x_addr, NULL, 16);
#ifdef DEBUG
			printf("\n** bootusb: FLASHING from %x\n", addr);
#else
			printf("\n");
#endif
		}
		else if (!strcmp(cmd, "FLSH")) {
			memset(x_addr, 0, sizeof(x_addr));
			strncpy(x_addr, (char*)p, 8);
			addr =  simple_strtoul(x_addr, NULL, 16);
#ifdef DEBUG
			printf("\n** bootusb: FLASHING to %x\n", addr);
#else
			printf("\n");
#endif
			bootusb_confirm();
			udelay(1000);
			udc_stop();

			img_tail = img_size % 0x4000;
			if (img_tail)
				img_size += 0x4000 - img_tail;
			sprintf(flash_script, "nand erase 0x%08x 0x%x; nand write 0x%08x 0x%08x 0x%x; reset",
				addr, img_size, img_start, addr, img_size);
#ifdef DEBUG
			printf("Running '%s'\n", flash_script);
#endif
			run_command(flash_script, 0);
			/* nobody seems to get here... */
			for (;;);
		}
		else if (!strcmp(cmd, "BIN")) {
			u8 sum;

			if (!addr) {
				printf("Error - no addr to load data!!\n");
				continue;
			}
			chunk_size = ntohs(((u16*)p)[0]);
			memcpy((void*)addr, p + sizeof(u16), chunk_size);
			sum = ezx_csum(p + sizeof(u16), chunk_size );
#ifdef DEBUG
			printf("Chunk %d/%d bytes\n", chunk_size, size);
			printf("CS = %02X, expected = %02X\n", sum, p[size-1]);
#else
			printf("#");
			if ((++count) % 50 == 0)
				printf("\n\t");
#endif
			if (sum != p[size-1]) {
				printf("F A T A L! Checksum mismatch\n");
				hang();
			}
			img_size += chunk_size;
		}
		else {
			printf("Error - unknown command '%s'\n", cmd);
		}

		if (p && size)
			free(p);
		bootusb_confirm();

	} while (1);
}

/*********************************************************************************/

/*********************************************************************************/

/*
 * Since interrupt handling has not yet been implemented, we use this function
 * to handle polling.  This is called by the tstc,getc,putc,puts routines to
 * update the USB state.
 */
void bootusb_poll (void)
{
	/* New interrupts? */
	udc_poll_irq();
	if (bootusb_configured())
		fill_buffer(&usb_input);
	/* again - any news ? */
	udc_poll_irq();
}

#endif

