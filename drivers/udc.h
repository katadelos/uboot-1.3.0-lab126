#ifndef ___USBDCORE_H
#define ___USBDCORE_H

void udc_init(void);
void udc_endpoint_write (struct usb_endpoint_instance *endpoint);
void udc_setup_ep (struct usb_device_instance *device,
                   unsigned int ep_num, struct usb_endpoint_instance *endpoint);
void udc_connect (void);
void udc_disconnect (void);
void udc_enable (struct usb_device_instance *device);
void udc_disable (void);
void udc_startup_events (struct usb_device_instance *device);
void udc_init(void);
void udc_poll_irq(void);

#endif
