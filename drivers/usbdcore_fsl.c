#include <common.h>

#if defined(CONFIG_DRIVER_FSLUSB) && defined(CONFIG_USB_DEVICE)

#define DEBUG

#include <malloc.h>
#include <usbdcore.h>
#include <asm/arch/usb.h>
#include "udc.h"
#include "usbdcore_fsl.h"
#include "usbdcore_ep0.h"

#define ep_index(ep) \
		((ep)->endpoint_address & USB_ENDPOINT_NUMBER_MASK)
#define __ep_is_in(udc, num) ( (num == 0) ? ((udc)->ep0_dir == \
                        USB_DIR_IN ):(num \
                        & USB_DIR_IN)==USB_DIR_IN)

#define ep_is_in(udc,EP)   __ep_is_in(udc,(EP)->endpoint_address)

struct DTD {
	u32	next;
	u32 	flags;
	u32	ptr0;
	u32	ptr1;
	u32	ptr2;
	u32	ptr3;
	u32 	ptr4;
	u32	reserved;

	u8 reserved_2[0x100 - sizeof(u32) * 8];
};

#define USB_DTD_MAX 5
#define DTD_DATA_LEN 0x4000

static u8 *dtd_array;
#define DTD_ITEM(item) (struct DTD*)(dtd_array + sizeof(struct DTD) * item)
static struct DTD* dtds[USB_DTD_MAX];
static u8* dtd_in_buffer;
static u8* dtd_data_buffer;

static struct usb_dr_device *dr_regs = (struct usb_dr_device*)USB_BASE;
static struct urb *ep0_urb = NULL;

static struct usb_device_instance *udc_device;	/* Used in interrupt handler */
static u16 udc_devstat = 0;	/* UDC status (DEVSTAT) */
static u32 udc_interrupts = 0;
static struct fsl_udc fsl_udc;
static void udc_stall_ep (unsigned int ep_addr);
static void fsl_configure_device (struct usb_device_instance *device);
static void fsl_deconfigure_device (void);

static struct DTD* dtd_setup(struct DTD* dtd, u8* buffer, int len, int in)
{
	struct DTD *c =dtd;
	u32 bptr;

	if (in) {
		bptr = (u32)dtd_in_buffer;
		len = DTD_DATA_LEN;
	} else  {
		memcpy(dtd_data_buffer, buffer, len);
		bptr = (u32)dtd_data_buffer;
	}

	c->ptr0 = bptr;
	c->ptr1 = bptr + 0x1000;
	c->ptr2 = bptr + 0x2000;
	c->ptr3 = bptr + 0x3000;
	c->ptr4 = bptr + 0x4000;
	c->flags = (len << DTD_LENGTH_BIT_POS);
	c->flags |= DTD_STATUS_ACTIVE | DTD_IOC;
	c->next = 1;

	return dtd;
}

static struct usb_endpoint_instance *fsl_find_ep (int ep)
{
        int i;

        for (i = 0; i < udc_device->bus->max_endpoints; i++) {
                if (udc_device->bus->endpoint_array[i].endpoint_address == ep) {
                        return &udc_device->bus->endpoint_array[i];
		}
        }
        return NULL;
}

static void queue_dtd(struct fsl_udc* udc, struct usb_endpoint_instance* ep, struct DTD *dtd)
{
	u32 bitmask, temp, tmp_stat;
	int dir = ep_is_in(udc,ep) ? 1 : 0;
	int ep_num = ep_index(ep);
        struct ep_queue_head *qh = &udc->ep_qh[2 * ep_num + dir];

	bitmask = ep_is_in(udc,ep)
                ? (1 << (ep_index(ep) + 16))
                : (1 << (ep_index(ep)));

        /* Read prime bit, if 1 goto done */
        if (fsl_readl(&dr_regs->endpointprime) & bitmask) {
                goto out;
	}

        do {
                /* Set ATDTW bit in USBCMD */
                temp = fsl_readl(&dr_regs->usbcmd);
                fsl_writel(temp | USB_CMD_ATDTW, &dr_regs->usbcmd);

                /* Read correct status bit */
                tmp_stat = fsl_readl(&dr_regs->endptstatus) & bitmask;

        } while (!(fsl_readl(&dr_regs->usbcmd) & USB_CMD_ATDTW));

        /* Write ATDTW bit to 0 */
        temp = fsl_readl(&dr_regs->usbcmd);
        fsl_writel(temp & ~USB_CMD_ATDTW, &dr_regs->usbcmd);

        if (tmp_stat)
               goto out;

	temp = (u32)dtd;
	udc->saved_dtd[ep_num * 2 + dir] = dtd; 		/* save the 1st DTD in list */
        qh->next_dtd_ptr = cpu_to_le32(temp) & 0xFFFFFFE0;

        /* Clear active and halt bit */
        temp = cpu_to_le32(~(EP_QUEUE_HEAD_STATUS_ACTIVE
                        | EP_QUEUE_HEAD_STATUS_HALT));
        qh->size_ioc_int_sts &= temp;

        /* Prime endpoint by writing 1 to ENDPTPRIME */
	temp = fsl_readl(&dr_regs->endpointprime);
        fsl_writel(temp | bitmask, &dr_regs->endpointprime);

        /* Update ep0 state */
        if (ep_num == 0)
                udc->ep0_state = DATA_STATE_XMIT;
#ifdef DEBUG
	if (ep_num != 0)
		serial_printf("# %x = %x @ %d / QH=%x\n", ep_num, bitmask, ep_num * 2 + dir, qh);
#endif

out:
	return;
}

/* Prime a status phase for ep0 */
static int ep0_prime_status(struct fsl_udc *udc, int direction)
{
	udc->ep0_dir = direction;
        udc->ep0_state = WAIT_FOR_OUT_STATUS;
	queue_dtd(udc, fsl_find_ep(0), dtd_setup(dtds[0],NULL,0,0));
        return 0;
}


/*
 * Start of public functions.
 */

/* Called to start packet transmission. */
void udc_endpoint_write (struct usb_endpoint_instance *endpoint)
{
	queue_dtd(&fsl_udc, endpoint, dtd_setup(dtds[1],endpoint->tx_urb->buffer, endpoint->tx_urb->actual_length, 0));
}

static void
dr_ep_change_stall(unsigned char ep_num, unsigned char dir, int value)
{
	u32 tmp_epctrl = 0;

	tmp_epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);

	if (value) {
		/* set the stall bit */
		if (dir)
			tmp_epctrl |= EPCTRL_TX_EP_STALL;
		else
			tmp_epctrl |= EPCTRL_RX_EP_STALL;
		if (ep_num == 0)
			tmp_epctrl |= EPCTRL_RX_EP_STALL | EPCTRL_TX_EP_STALL;
	} else {
		/* clear the stall bit and reset data toggle */
		if (dir) {
			tmp_epctrl &= ~EPCTRL_TX_EP_STALL;
			tmp_epctrl |= EPCTRL_TX_DATA_TOGGLE_RST;
		} else {
			tmp_epctrl &= ~EPCTRL_RX_EP_STALL;
			tmp_epctrl |= EPCTRL_RX_DATA_TOGGLE_RST;
		}
	}
	fsl_writel(tmp_epctrl, &dr_regs->endptctrl[ep_num]);
}

/* Stall endpoint */
static void udc_stall_ep (unsigned int ep_addr)
{
	dr_ep_change_stall(ep_addr & 0x7f, __ep_is_in(&fsl_udc, ep_addr), 1);
}

/*
 * udc_eetup_ep - setup endpoint
 *
 * Associate a physical endpoint with endpoint_instance
 */
static void dr_ep_setup(unsigned char ep_num, unsigned char dir, unsigned char ep_type)
{
        unsigned int tmp_epctrl = 0;

        tmp_epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);
        if (dir) {
                if (ep_num)
                        tmp_epctrl |= EPCTRL_TX_DATA_TOGGLE_RST;
                tmp_epctrl |= EPCTRL_TX_ENABLE;
                tmp_epctrl |= ((unsigned int)(ep_type)
                                << EPCTRL_TX_EP_TYPE_SHIFT);
        } else {
                if (ep_num)
                        tmp_epctrl |= EPCTRL_RX_DATA_TOGGLE_RST;
                tmp_epctrl |= EPCTRL_RX_ENABLE;
                tmp_epctrl |= ((unsigned int)(ep_type)
                                << EPCTRL_RX_EP_TYPE_SHIFT);
        }

        fsl_writel(tmp_epctrl, &dr_regs->endptctrl[ep_num]);

}

static void struct_ep_qh_setup(struct fsl_udc* udc, unsigned char ep_num,
                unsigned char dir, unsigned char ep_type,
                unsigned int max_pkt_len,
                unsigned int zlt, unsigned char mult)
{
        struct ep_queue_head *p_QH;
        unsigned int tmp = 0;

	dir = dir ? 1 : 0;
 	p_QH = &udc->ep_qh[2 * ep_num + dir];

        /* set the Endpoint Capabilites in QH */
        switch (ep_type) {
        case USB_ENDPOINT_XFER_CONTROL:
                /* Interrupt On Setup (IOS). for control ep  */
                tmp = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS)
                        | EP_QUEUE_HEAD_IOS;
                break;
        case USB_ENDPOINT_XFER_ISOC:
                tmp = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS)
                        | (mult << EP_QUEUE_HEAD_MULT_POS);
                break;
        case USB_ENDPOINT_XFER_BULK:
        case USB_ENDPOINT_XFER_INT:
                tmp = max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS;
                break;
        default:
                VDBG("error ep type is %d", ep_type);
                return;
        }

        if (zlt)
                tmp |= EP_QUEUE_HEAD_ZLT_SEL;
        p_QH->max_pkt_length = cpu_to_le32(tmp);

        return;
}


void udc_setup_ep (struct usb_device_instance *device,
		   unsigned int ep_num, struct usb_endpoint_instance *endpoint)
{
        unsigned short max = 0;
        unsigned char mult = 0, zlt;

        max = endpoint->rcv_packetSize;

        /* Disable automatic zlp generation.  Driver is reponsible to indicate
         * explicitly through req->req.zero.  This is needed to enable multi-td
         * request. */
        zlt = 1;

        /* Assume the max packet size from gadget is always correct */
        switch (endpoint->tx_attributes & 0x03)
	{
        case USB_ENDPOINT_XFER_BULK:
		zlt = 0;
        case USB_ENDPOINT_XFER_CONTROL:
        case USB_ENDPOINT_XFER_INT:
                /* mult = 0.  Execute N Transactions as demonstrated by
                 * the USB variable length packet protocol where N is
                 * computed using the Maximum Packet Length (dQH) and
                 * the Total Bytes field (dTD) */
                mult = 0;
                break;
        case USB_ENDPOINT_XFER_ISOC:
                /* Calculate transactions needed for high bandwidth iso */
                mult = (unsigned char)(1 + ((max >> 11) & 0x03));
                max = max & 0x8ff;      /* bit 0~10 */
                /* 3 transactions at most */
                if (mult > 3)
                        goto en_done;
                break;
        default:
                goto en_done;
        }

	if( ep_num!=0)  {
		struct_ep_qh_setup(&fsl_udc, (unsigned char)ep_num,
                        (unsigned char) ((endpoint->endpoint_address & USB_DIR_IN)
                                        ? USB_RECV: USB_SEND),
                        (unsigned char) (endpoint->tx_attributes
                                        & USB_ENDPOINT_XFERTYPE_MASK),
                        max, zlt, mult);
        	dr_ep_setup((unsigned char)ep_num,
                        (unsigned char) ((endpoint->endpoint_address & USB_DIR_IN)
                                        ? USB_RECV: USB_SEND),
                        (unsigned char) (endpoint->tx_attributes
                                        & USB_ENDPOINT_XFERTYPE_MASK));
	}
	return;
en_done:
	serial_printf("%s: SOMETHING IS BAD!\n", __FUNCTION__);
}

/* Turn on the USB connection by enabling the pullup resistor */
void udc_connect (void)
{
        fsl_writel((fsl_readl(&dr_regs->usbcmd) | USB_CMD_RUN_STOP),
                                &dr_regs->usbcmd);
}

/* Turn off the USB connection by disabling the pullup resistor */
void udc_disconnect (void)
{
        fsl_writel((fsl_readl(&dr_regs->usbcmd) & ~USB_CMD_RUN_STOP),
                                &dr_regs->usbcmd);
}

void udc_enable (struct usb_device_instance *device)
{
	/* initialize driver state variables */
	udc_devstat = 0;

	/* Save the device structure pointer */
	udc_device = device;

	/* Setup ep0 urb */
	if (ep0_urb)
		usbd_dealloc_urb(ep0_urb);
	ep0_urb = usbd_alloc_urb (udc_device,
		udc_device->bus->endpoint_array);
	fsl_configure_device (device);
}

/**
 * Switch off the UDC
 */
void udc_disable (void)
{
	fsl_deconfigure_device();
	/* Free ep0 URB */
	if (ep0_urb) {
		usbd_dealloc_urb(ep0_urb);
		ep0_urb = NULL;
	}
        fsl_writel(0, &dr_regs->usbintr);
}

/**
 * udc_startup - allow udc code to do any additional startup
 */
void udc_startup_events (struct usb_device_instance *device)
{
	usbd_device_event_irq (device, DEVICE_INIT, 0);
	usbd_device_event_irq (device, DEVICE_CREATE, 0);
	udc_enable (device);
}

static void struct_udc_setup(void)
{
	struct fsl_udc *udc = &fsl_udc;

	udc->max_ep = 4;
	udc->ep_qh = memalign(0x100, 8 * sizeof(struct ep_queue_head));	/* FIXME */
#ifdef DEBUG
	serial_printf("QUEUE HEAD %p\n", udc->ep_qh);
#endif
	udc->usb_state = STATE_POWERED;
	udc->ep0_dir = USB_DIR_IN; /* was 0 */
	udc->remote_wakeup = 0;	/* default to 0 on reset */
}

static int dr_controller_setup(struct fsl_udc *udc)
{
	unsigned int tmp = 0, portctrl = 0;

	/* Stop and reset the usb controller */
	tmp = fsl_readl(&dr_regs->usbcmd);
	tmp &= ~USB_CMD_RUN_STOP;
	fsl_writel(tmp, &dr_regs->usbcmd);

	tmp = fsl_readl(&dr_regs->usbcmd);
	tmp |= USB_CMD_CTRL_RESET;
	fsl_writel(tmp, &dr_regs->usbcmd);

	/* Wait for reset to complete */
	while (fsl_readl(&dr_regs->usbcmd) & USB_CMD_CTRL_RESET) {
		continue;
	}

	/* Set the controller as device mode */
	tmp = fsl_readl(&dr_regs->usbmode);
	tmp |= USB_MODE_CTRL_MODE_DEVICE;
	/* Disable Setup Lockout */
	tmp |= USB_MODE_SETUP_LOCK_OFF;
	fsl_writel(tmp, &dr_regs->usbmode);

	/* Clear the setup status */
	fsl_writel(0, &dr_regs->usbsts);

	tmp = (u32)udc->ep_qh;
	tmp &= USB_EP_LIST_ADDRESS_MASK;
	fsl_writel(tmp, &dr_regs->endpointlistaddr);

	/* Config PHY interface */
	portctrl = fsl_readl(&dr_regs->portsc1);
	portctrl &= ~(PORTSCX_PHY_TYPE_SEL & PORTSCX_PORT_WIDTH);
        portctrl |= PORTSCX_PTS_ULPI;

	fsl_writel(portctrl, &dr_regs->portsc1);

	return 0;
}

void udc_init(void)
{
	int i;

	fsl_start_usb();
	dtd_in_buffer = memalign(0x100, DTD_DATA_LEN);
	dtd_data_buffer = memalign(0x100, DTD_DATA_LEN);
	dtd_array = memalign(0x100, sizeof(struct DTD) * USB_DTD_MAX);
	for (i = 0; i < USB_DTD_MAX; i ++)
		dtds[i] = dtd_array + sizeof(struct DTD);
#ifdef DEBUG
	serial_printf("Allocated: in = %p, data = %p, dtd array = %p\n", dtd_in_buffer, dtd_data_buffer, dtd_array);
#endif
}

void udc_stop(void)
{
	fsl_stop_usb();
	free(dtd_in_buffer);
	free(dtd_data_buffer);
	free(dtd_array);
}

/* Enable DR irq and set controller to run state */
static void dr_controller_run(struct fsl_udc *udc)
{
        u32 temp;

        /* Enable DR irq reg */
        temp = USB_INTR_INT_EN | USB_INTR_ERR_INT_EN
                | USB_INTR_PTC_DETECT_EN | USB_INTR_RESET_EN
                | USB_INTR_DEVICE_SUSPEND | USB_INTR_SYS_ERR_EN;

        fsl_writel(temp, &dr_regs->usbintr);

        /* Clear stopped bit */
        udc->stopped = 0;

        /* Set the controller as device mode */
        temp = fsl_readl(&dr_regs->usbmode);
        temp |= USB_MODE_CTRL_MODE_DEVICE;
        fsl_writel(temp, &dr_regs->usbmode);

        /* Set controller to Run */
        temp = fsl_readl(&dr_regs->usbcmd);
        temp |= USB_CMD_RUN_STOP;
        fsl_writel(temp, &dr_regs->usbcmd);

        return;
}

/* Setup qh structure and ep register for ep0. */
static void ep0_setup(struct fsl_udc *udc)
{
        /* the intialization of an ep includes: fields in QH, Regs,
         * fsl_ep struct */
        struct_ep_qh_setup(udc, 0, USB_RECV, USB_ENDPOINT_XFER_CONTROL,
                        USB_MAX_CTRL_PAYLOAD, 0, 0);
        struct_ep_qh_setup(udc, 0, USB_SEND, USB_ENDPOINT_XFER_CONTROL,
                        USB_MAX_CTRL_PAYLOAD, 0, 0);
        dr_ep_setup(0, USB_RECV, USB_ENDPOINT_XFER_CONTROL);
        dr_ep_setup(0, USB_SEND, USB_ENDPOINT_XFER_CONTROL);

        return;
}

static void fsl_configure_device (struct usb_device_instance *device)
{
	u32 temp;

	struct_udc_setup();

        /* Enable DR irq reg */
        temp = USB_INTR_INT_EN | USB_INTR_ERR_INT_EN
                | USB_INTR_PTC_DETECT_EN | USB_INTR_RESET_EN
                | USB_INTR_DEVICE_SUSPEND | USB_INTR_SYS_ERR_EN;

        fsl_writel(temp, &dr_regs->usbintr);

        /* Set the controller as device mode */
        temp = fsl_readl(&dr_regs->usbmode);
        temp |= USB_MODE_CTRL_MODE_DEVICE;
        fsl_writel(temp, &dr_regs->usbmode);

	ep0_setup(&fsl_udc);
}

static void fsl_deconfigure_device ()
{
	struct fsl_udc *udc = &fsl_udc;

	free(udc->ep_qh);
}

/* Process a port change interrupt */
static void port_change_irq(struct fsl_udc *udc)
{
	u32 sc1;

	sc1 = fsl_readl(&dr_regs->portsc1);

        /* Bus resetting is finished */
        if (!(sc1 & PORTSCX_PORT_RESET)) {
		usbd_device_event_irq(udc_device, DEVICE_RESET, 0);
		return;
        }
}

static void suspend_irq(struct fsl_udc *udc)
{
	usbd_device_event_irq (udc_device, DEVICE_BUS_INACTIVE, 0);
}

/* Process reset interrupt */
static void reset_irq(struct fsl_udc *udc)
{
        u32 temp;

        temp = fsl_readl(&dr_regs->portsc1);
        /* not suspended? */
        if (temp & PORTSCX_PORT_SUSPEND) {
	        temp |= PORTSCX_PORT_FORCE_RESUME;
	        fsl_writel(temp, &dr_regs->portsc1);
	}

        /* Clear the device address */
        temp = fsl_readl(&dr_regs->deviceaddr);
        fsl_writel(temp & ~USB_DEVICE_ADDRESS_MASK, &dr_regs->deviceaddr);

        /* Clear usb state */
        udc->resume_state = 0;
        udc->ep0_dir = USB_DIR_IN; /* was 0 */
        udc->ep0_state = WAIT_FOR_SETUP;
        udc->remote_wakeup = 0; /* default to 0 on reset */

        /* Clear all the setup token semaphores */
        temp = fsl_readl(&dr_regs->endptsetupstat);
        fsl_writel(temp, &dr_regs->endptsetupstat);

        /* Clear all the endpoint complete status bits */
        temp = fsl_readl(&dr_regs->endptcomplete);
        fsl_writel(temp, &dr_regs->endptcomplete);

        while (fsl_readl(&dr_regs->endpointprime))
		continue;
	       /* Write 1s to the flush register */
        fsl_writel(0xffffffff, &dr_regs->endptflush);

        if (fsl_readl(&dr_regs->portsc1) & PORTSCX_PORT_RESET) {
                VDBG("Bus reset");
                /* Bus is reseting */
                udc->bus_reset = 1;
                /* Reset all the queues, include XD, dTD, EP queue
                 * head and TR Queue */
                udc->usb_state = STATE_DEFAULT;
        } else {
                VDBG("Controller reset");
                /* initialize usb hw reg except for regs for EP, not
                 * touch usbintr reg */
                dr_controller_setup(udc);

                /* Reset all internal used Queues */
                ep0_setup(udc);

                /* Enable DR IRQ reg, Set Run bit, change udc state */
                dr_controller_run(udc);
                udc->usb_state = STATE_ATTACHED;
        }
}

/* Tripwire mechanism to ensure a setup packet payload is extracted without
 * being corrupted by another incoming setup packet */
static void tripwire_handler(struct fsl_udc *udc, u8 ep_num, u8 *buffer_ptr)
{
        u32 temp;
        struct ep_queue_head *qh;

        qh = &udc->ep_qh[ep_num * 2 + EP_DIR_OUT];

        /* Clear bit in ENDPTSETUPSTAT */
        temp = fsl_readl(&dr_regs->endptsetupstat);
        fsl_writel(temp | (1 << ep_num), &dr_regs->endptsetupstat);

        /* while a hazard exists when setup package arrives */
        do {
                /* Set Setup Tripwire */
                temp = fsl_readl(&dr_regs->usbcmd);
                fsl_writel(temp | USB_CMD_SUTW, &dr_regs->usbcmd);

                /* Copy the setup packet to local buffer */
                memcpy(buffer_ptr, (u8 *) qh->setup_buffer, 8);

        } while (!(fsl_readl(&dr_regs->usbcmd) & USB_CMD_SUTW));

        /* Clear Setup Tripwire */
        temp = fsl_readl(&dr_regs->usbcmd);
        fsl_writel(temp & ~USB_CMD_SUTW, &dr_regs->usbcmd);
}

static inline int udc_reset_ep_queue(struct fsl_udc *udc, u8 pipe)
{
	return 0;
}

static void setup_received_irq(struct fsl_udc *udc,
                struct urb *urb)
{
        udc_reset_ep_queue(udc, 0);

	urb->actual_length = 0;
	if (ep0_recv_setup(urb)) {
		udc_stall_ep(0x00);
                udc->ep0_state = (urb->device_request.bmRequestType & USB_DIR_IN)
                         ?  DATA_STATE_XMIT : DATA_STATE_RECV;
	}

        udc->ep0_dir = (ep0_urb->device_request.bmRequestType & USB_DIR_IN)
                                        ?  USB_DIR_IN : USB_DIR_OUT;
       /* Check direction */
        if ((ep0_urb->device_request.bmRequestType & USB_REQ_DIRECTION_MASK)
            == USB_REQ_HOST2DEVICE) {
                if (le16_to_cpu (ep0_urb->device_request.wLength)) {
			udc_stall_ep(0x00);
		} else {
			ep0_prime_status(udc, USB_DIR_IN);
		}
	} else {
		queue_dtd(udc, fsl_find_ep(0), dtd_setup(dtds[0], ep0_urb->buffer, ep0_urb->actual_length, 0));
	}

	switch (ep0_urb->device_request.bRequest)
	{
	case USB_REQ_SET_ADDRESS:
		usbd_device_event_irq (udc_device, DEVICE_ADDRESS_ASSIGNED, udc_device->address);
		break;
	case USB_REQ_SET_INTERFACE:
		usbd_device_event_irq (udc_device, DEVICE_SET_INTERFACE, 0);
		break;
	case USB_REQ_SET_CONFIGURATION:
		usbd_device_event_irq(udc_device, DEVICE_CONFIGURED, 0);
                queue_dtd( udc, fsl_find_ep(0x2), dtd_setup(dtds[2], NULL, DTD_DATA_LEN, 1));
		break;
	}
}


void dtd_complete_irq(struct fsl_udc *udc)
{
        u32 bit_pos;
        int i, ep_num, direction, bit_mask;
	struct usb_endpoint_instance* ep;

        /* Clear the bits in the register */
        bit_pos = fsl_readl(&dr_regs->endptcomplete);
        fsl_writel(bit_pos, &dr_regs->endptcomplete);

#ifdef DEBUG
	printf("complete %x\n", bit_pos);
#endif
        if (!bit_pos)
                return;

        for (i = 0; i < udc->max_ep * 2; i++) {
                ep_num = i >> 1;
                direction = i % 2;

                bit_mask = 1 << (ep_num + 16 * direction);

                if (!(bit_pos & bit_mask))
                        continue;

#ifdef DEBUG
		printf("Got ep_num = %d, direction = %d\n", ep_num, direction);
#endif
		ep = fsl_find_ep((direction << 7) | ep_num);

		if (i > 1) /* non-control ep */ {

			if (NULL == ep)	{
				printf("F A T A L - can't find ep with address %x\n", (direction << 7) | ep_num);
			}

			if (direction == 0) /* host-to-device, input in our case */ {

				ep->rcv_urb->actual_length = DTD_DATA_LEN - (dtds[2]->flags >> 16);
				memcpy(ep->rcv_urb->buffer, dtd_in_buffer, ep->rcv_urb->actual_length);

	                	queue_dtd( udc, fsl_find_ep((direction << 7) | ep_num),
					dtd_setup(dtds[2], NULL, DTD_DATA_LEN, 1));
			}
		}

		else

		{
			if (udc_device->address) {
				/* device is addressed ? */
		                fsl_writel(udc_device->address << USB_DEVICE_ADDRESS_BIT_POS,
                             	   &dr_regs->deviceaddr);

			}
			switch (udc->ep0_state)
			{
			case DATA_STATE_XMIT:
				 ep0_prime_status(udc, USB_DIR_OUT);
				 break;
			case DATA_STATE_RECV:
				 ep0_prime_status(udc, USB_DIR_IN);
				 break;
		        case WAIT_FOR_OUT_STATUS:
                	 	 udc->ep0_state = WAIT_FOR_SETUP;
				 break;
			default:
				 printf("Unknown state %x\n", udc->ep0_state);
				 break;
			}
		}
        }
}

void udc_poll_irq(void)
{
	u32 irq_src;
	struct fsl_udc* udc = &fsl_udc;
	u32 eps;

	for(;;) {
	}

	for(;;) {
        	irq_src = fsl_readl(&dr_regs->usbsts);
	        /* Clear notification bits */
 		irq_src &= fsl_readl(&dr_regs->usbintr);
	        fsl_writel(irq_src, &dr_regs->usbsts);

		if (!(irq_src & ~0x80))
			break;

#ifdef DEBUG
		printf("IRQ: %x\n",irq_src);
#endif
     	   	/* USB Interrupt */
        	if (irq_src & USB_STS_INT) {
	                /* Setup package, we only support ep0 as control ep */
			eps = 	fsl_readl(&dr_regs->endptsetupstat);
	                if (eps & EP_SETUP_STATUS_EP0) {
	                        tripwire_handler(udc, 0,
	                                        (u8 *)&ep0_urb->device_request );
   	                        setup_received_irq(udc, ep0_urb);
                	}

        	        /* completion of dtd */
			eps = fsl_readl(&dr_regs->endptcomplete);
                	if (eps)
                        	dtd_complete_irq(udc);
			irq_src &= ~USB_STS_INT;
                }

	        /* Port Change */
	        if (irq_src & USB_STS_PORT_CHANGE) {
	                port_change_irq(udc);
			irq_src &= ~USB_STS_PORT_CHANGE;
	        }

	        /* Reset Received */
	        if (irq_src & USB_STS_RESET) {
	                reset_irq(udc);
			irq_src &= ~USB_STS_RESET;
			usbd_device_event_irq (udc_device, DEVICE_RESET, 0);
	        }

	        /* Sleep Enable (Suspend) */
	        if (irq_src & USB_STS_SUSPEND) {
	                suspend_irq(udc);
			irq_src &= ~USB_STS_SUSPEND;
	        }
#ifdef DEBUG
		if (irq_src != 0)
			printf("IRQ_SRC = %x\n", irq_src);

        	if (irq_src & (USB_STS_ERR | USB_STS_SYS_ERR)) {
	                printf("Error IRQ %x ", irq_src);
	        }
#endif
	}
}


#endif
