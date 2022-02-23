// Code adapted from:
// https://github.com/libopencm3/libopencm3-examples/tree/master/examples/stm32/f1/stm32-h107/usb_simple
#ifndef USBD_SETUP_H
#define USBD_SETUP_H
#include <libopencm3/usb/usbd.h>

// Constants:
static const uint8_t BULK_BUFFER_SIZE = 64;
static const uint8_t CTRL_BUFFER_SIZE = 128;

extern usbd_device* usbd_dev; // Called in ISR and main loop.
extern uint8_t usbd_control_buffer[CTRL_BUFFER_SIZE];
extern uint8_t bulk_buf[BULK_BUFFER_SIZE];

extern const struct usb_device_descriptor dev;
extern const struct usb_endpoint_descriptor data_endp[];
extern const struct usb_interface_descriptor iface;
extern const struct usb_interface ifaces[];
extern const struct usb_config_descriptor config;
extern const char* usb_strings[];


#endif //USBD_SETUP_H
