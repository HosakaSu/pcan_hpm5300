#include <string.h>
#include "board.h"
#include "usb_config.h"
#include "usbd_core.h"
#include "pcan_led.h"
#include "pcan_usb.h"

/*
 * Phase 1: CherryUSB "vendor" device that reproduces the PCAN-USB descriptor
 * set of the reference STM32 firmware (see pcan_cantact/Src/pcan_usb.c and
 * usbd_desc.c), but with VID/PID overridden to PEAK-System's PCAN-USB IDs
 * (0x0c72 / 0x000c). The HPM5301 DWC2 port negotiates full speed only, so a
 * single full-speed configuration descriptor is returned for any speed.
 */

#define PCAN_USB_BUSID        0
#define PCAN_USB_CONFIG_SIZE  (9 + 9 + 7 * 4)   /* config + intf + 4 endpoints = 46 */

/* ============================ descriptors ============================ */

/* Hand-built so iSerial stays 0 (the shared macro would force it to 3). */
static const uint8_t device_descriptor[] = {
    0x12,                          /* bLength */
    USB_DESCRIPTOR_TYPE_DEVICE,    /* bDescriptorType */
    0x00, 0x01,                    /* bcdUSB = 0x0100 (USB 1.0) */
    0x00,                          /* bDeviceClass */
    0x00,                          /* bDeviceSubClass */
    0x00,                          /* bDeviceProtocol */
    0x40,                          /* bMaxPacketSize0 = 64 */
    0x72, 0x0c,                    /* idVendor  = 0x0c72 (PEAK-System) */
    0x0c, 0x00,                    /* idProduct = 0x000c (PCAN-USB) */
    0xff, 0x54,                    /* bcdDevice = 0x54ff */
    0x01,                          /* iManufacturer */
    0x02,                          /* iProduct */
    0x00,                          /* iSerial = 0 (none) */
    0x01,                          /* bNumConfigurations */
};

/* Bus-powered, MaxPower raw = USB_CONFIG_POWER_MA(200) = 100 (200 mA). */
static const uint8_t config_descriptor_fs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(PCAN_USB_CONFIG_SIZE, 1, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    USB_INTERFACE_DESCRIPTOR_INIT(0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(PCAN_USB_EP_CMDIN,  USB_ENDPOINT_TYPE_BULK, 16, 0x00), /* cmd IN  */
    USB_ENDPOINT_DESCRIPTOR_INIT(PCAN_USB_EP_CMDOUT, USB_ENDPOINT_TYPE_BULK, 16, 0x00), /* cmd OUT */
    USB_ENDPOINT_DESCRIPTOR_INIT(PCAN_USB_EP_MSGIN,  USB_ENDPOINT_TYPE_BULK, 64, 0x00), /* msg IN  */
    USB_ENDPOINT_DESCRIPTOR_INIT(PCAN_USB_EP_MSGOUT, USB_ENDPOINT_TYPE_BULK, 64, 0x00), /* msg OUT */
};

static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 },   /* index 0: Langid 0x0409 */
    "PEAK-System Technik GmbH",     /* index 1: Manufacturer */
    "PCAN-USB",                     /* index 2: Product */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    /* FS-only device: the same full-speed config is returned for any speed. */
    (void)speed;
    return config_descriptor_fs;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    /* bcdUSB 1.0 device: hosts do not use the device-qualifier descriptor. */
    (void)speed;
    return NULL;
}

static const uint8_t *other_speed_config_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return NULL;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;
    if (index >= (sizeof(string_descriptors) / sizeof(char *))) {
        return NULL;
    }
    return string_descriptors[index];
}

static const struct usb_descriptor pcan_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .other_speed_descriptor_callback = other_speed_config_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

/* ==================== endpoint rx buffers (DMA-safe) ==================== */

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t pcan_cmd_rx_buf[16];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t pcan_msg_rx_buf[64];

static volatile bool pcan_cmdin_busy;
static volatile bool pcan_msgin_busy;

/* ======================= vendor request handler ======================== */

/*
 * Mirrors device_setup() in the reference firmware. CherryUSB interface
 * handler convention (see usbd_core.h): set *data / *len and return 0 on
 * success, return non-zero (negative) to STALL the control request.
 */
static int usbd_pcan_vendor_handler(uint8_t busid, struct usb_setup_packet *setup, uint8_t **data, uint32_t *len)
{
    (void)busid;

    if ((setup->bmRequestType & USB_REQUEST_TYPE_MASK) != USB_REQUEST_VENDOR) {
        return -1;
    }

    /* vendor + IN + bRequest==0 + wValue==0 -> bootloader info */
    if ((setup->bmRequestType & USB_REQUEST_DIR_MASK) == USB_REQUEST_DIR_IN &&
        setup->bRequest == 0 && setup->wValue == 0) {
        static uint8_t bootloader_info[16] = {
            0x00, 0x00, 0x08, 0x04, 0x00, 0x08, 0x07, 0x00,
            0x04, 0x02, 0xe0, 0x07, 0x01, 0x00, 0x00, 0x00
        };
        *data = bootloader_info;
        *len = sizeof(bootloader_info);
        return 0;
    }

    /* Phase 2: PCAN_USB_PARAM get/set. STALL everything else for now. */
    return -1;
}

/* ========================= endpoint callbacks ========================== */

static void usbd_pcan_cmdout(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("pcan cmd out len:%d\r\n", (int)nbytes);
    /* Phase 2: pcan_protocol_process_command(pcan_cmd_rx_buf, nbytes); */
    usbd_ep_start_read(busid, ep, pcan_cmd_rx_buf, sizeof(pcan_cmd_rx_buf));
}

static void usbd_pcan_msgout(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("pcan msg out len:%d\r\n", (int)nbytes);
    /* Phase 2: pcan_protocol_process_data(pcan_msg_rx_buf, nbytes); */
    usbd_ep_start_read(busid, ep, pcan_msg_rx_buf, sizeof(pcan_msg_rx_buf));
}

static void usbd_pcan_cmdin(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;
    pcan_cmdin_busy = false;
}

static void usbd_pcan_msgin(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;
    pcan_msgin_busy = false;
}

static struct usbd_endpoint pcan_cmdout_ep = {
    .ep_addr = PCAN_USB_EP_CMDOUT,
    .ep_cb = usbd_pcan_cmdout,
};

static struct usbd_endpoint pcan_cmdin_ep = {
    .ep_addr = PCAN_USB_EP_CMDIN,
    .ep_cb = usbd_pcan_cmdin,
};

static struct usbd_endpoint pcan_msgout_ep = {
    .ep_addr = PCAN_USB_EP_MSGOUT,
    .ep_cb = usbd_pcan_msgout,
};

static struct usbd_endpoint pcan_msgin_ep = {
    .ep_addr = PCAN_USB_EP_MSGIN,
    .ep_cb = usbd_pcan_msgin,
};

static struct usbd_interface intf0;

/* ============================ event handler ============================ */

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event) {
    case USBD_EVENT_CONFIGURED:
        pcan_cmdin_busy = false;
        pcan_msgin_busy = false;
        /* arm the first OUT reads */
        usbd_ep_start_read(busid, PCAN_USB_EP_CMDOUT, pcan_cmd_rx_buf, sizeof(pcan_cmd_rx_buf));
        usbd_ep_start_read(busid, PCAN_USB_EP_MSGOUT, pcan_msg_rx_buf, sizeof(pcan_msg_rx_buf));
        /* light the LED to indicate successful enumeration */
        pcan_led_set_mode(LED_CH0_TX, LED_MODE_ON, 0);
        break;

    case USBD_EVENT_RESET:
    case USBD_EVENT_CONNECTED:
    case USBD_EVENT_DISCONNECTED:
    case USBD_EVENT_SUSPEND:
    case USBD_EVENT_RESUME:
    case USBD_EVENT_SET_REMOTE_WAKEUP:
    case USBD_EVENT_CLR_REMOTE_WAKEUP:
    default:
        break;
    }
}

/* =============================== init ================================= */

void pcan_usb_init( void )
{
    intf0.intf_num = 0;
    intf0.vendor_handler = usbd_pcan_vendor_handler;

    usbd_desc_register(PCAN_USB_BUSID, &pcan_descriptor);
    usbd_add_interface(PCAN_USB_BUSID, &intf0);
    usbd_add_endpoint(PCAN_USB_BUSID, &pcan_cmdout_ep);
    usbd_add_endpoint(PCAN_USB_BUSID, &pcan_cmdin_ep);
    usbd_add_endpoint(PCAN_USB_BUSID, &pcan_msgout_ep);
    usbd_add_endpoint(PCAN_USB_BUSID, &pcan_msgin_ep);
    usbd_initialize(PCAN_USB_BUSID, CONFIG_HPM_USBD_BASE, usbd_event_handler);
}

void pcan_usb_poll( void )
{
    /* USB is serviced from the DWC2 interrupt on HPM5301; nothing to poll. */
}

/* ======================= Phase 2 stubs (return 0) ====================== */

int pcan_flush_ep( uint8_t ep )
{
    (void)ep;
    return 0;
}

int pcan_flush_data( struct t_m2h_fsm *pfsm, void *src, int size )
{
    (void)pfsm;
    (void)src;
    (void)size;
    return 0;
}

uint16_t pcan_usb_send_command_buffer( const void *p, uint16_t size )
{
    (void)p;
    (void)size;
    return 0;
}

uint16_t pcan_usb_send_data_buffer( const void *p, uint16_t size )
{
    (void)p;
    (void)size;
    return 0;
}
