#include <string.h>
#include "board.h"
#include "usb_config.h"
#include "usbd_core.h"
#include "pcan_led.h"
#include "pcan_protocol.h"
#include "pcan_usb.h"

/*
 * Phase 2: CherryUSB "vendor" device reproducing the PCAN-USB descriptor set of
 * the reference STM32 firmware (see pcan_cantact/Src/pcan_usb.c), with VID/PID
 * overridden to PEAK-System's PCAN-USB IDs (0x0c72 / 0x000c). The HPM5301 DWC2
 * port negotiates full speed only, so a single full-speed configuration
 * descriptor is returned for any speed.
 *
 * Transport mapping (reference STM32 HAL -> CherryUSB):
 *   USBD_LL_PrepareReceive  -> usbd_ep_start_read
 *   USBD_LL_Transmit        -> usbd_ep_start_write
 *   device_data_out         -> bulk OUT endpoint callbacks (IRQ context)
 *   device_data_in          -> bulk IN  endpoint callbacks (IRQ context)
 *   device_setup            -> intf0.vendor_handler (EP0)
 *   ep_in[].total_length / ep_tx_in_use[] -> pcan_ep_tx_in_use[]
 *
 * All four bulk endpoints are IRQ driven; the main loop only runs
 * pcan_protocol_poll(), which pumps the m2h flush FSM.
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

/*
 * Per-IN-endpoint TX state, indexed by (ep_addr & 0x0F). Replaces the reference
 * firmware's t_class_data.ep_tx_in_use[]/ep_tx_data_pending[] plus the STM32 HAL
 * ep_in[].total_length busy flag with a single pair of arrays.
 *   CMDIN  (0x81) -> index 1, used by pcan_usb_send_command_buffer()
 *   MSGIN  (0x82) -> index 2, used by pcan_flush_data() (m2h FSM)
 * These live in DLM (non-cacheable on HPM5301) and are only touched by the CPU.
 */
static volatile uint8_t  pcan_ep_tx_in_use[16];
static volatile uint16_t pcan_ep_tx_data_pending[16];

/* ======================= vendor request handler ======================== */

/*
 * Mirrors device_setup() in the reference firmware. The PCAN-USB only uses a
 * single EP0 vendor control request: an IN request with bRequest==0/wValue==0
 * that returns the 16-byte bootloader info. Every PCAN_USB_PARAM get/set
 * command travels over the bulk CMDOUT/CMDIN endpoints instead (handled in
 * pcan_protocol_process_command()), so there is no EP0 OUT data phase to
 * service here -- matching the reference (device_ep0_rx_ready() is empty).
 *
 * CherryUSB convention (usbd_core.h): set *data / *len and return 0 on success,
 * return a negative value to STALL the control request.
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

        /* reference device_setup() side effects: LEDs on + reset IN endpoints */
        pcan_led_set_mode(LED_CH0_RX, LED_MODE_ON, 0);
        pcan_led_set_mode(LED_CH0_TX, LED_MODE_ON, 0);
        pcan_flush_ep(PCAN_USB_EP_MSGIN);
        pcan_flush_ep(PCAN_USB_EP_CMDIN);
        return 0;
    }

    /* everything else: STALL (the reference never receives any other request) */
    return -1;
}

/* ========================= endpoint callbacks ========================== */

/*
 * Bulk OUT callbacks run in USB IRQ context (same model as the reference
 * firmware's device_data_out(), which is the STM32 USB ISR). Feed the payload
 * straight into the protocol layer and re-arm the read.
 */
static void usbd_pcan_cmdout(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    pcan_protocol_process_command(pcan_cmd_rx_buf, (uint16_t)nbytes);
    usbd_ep_start_read(busid, ep, pcan_cmd_rx_buf, sizeof(pcan_cmd_rx_buf));
}

static void usbd_pcan_msgout(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    pcan_protocol_process_data(pcan_msg_rx_buf, (uint16_t)nbytes);
    usbd_ep_start_read(busid, ep, pcan_msg_rx_buf, sizeof(pcan_msg_rx_buf));
}

/*
 * Bulk IN completion callbacks: mirror the reference device_data_in(), which
 * simply clears the "endpoint in use" flag. The m2h FSM keeps its state and is
 * advanced on the next pcan_flush_data() call from pcan_protocol_poll().
 */
static void usbd_pcan_cmdin(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)nbytes;
    pcan_ep_tx_in_use[ep & 0x0F] = 0;
}

static void usbd_pcan_msgin(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)nbytes;
    pcan_ep_tx_in_use[ep & 0x0F] = 0;
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
        memset((void *)pcan_ep_tx_in_use, 0, sizeof(pcan_ep_tx_in_use));
        memset((void *)pcan_ep_tx_data_pending, 0, sizeof(pcan_ep_tx_data_pending));
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

/* ====================== flush / send implementation ==================== */

/*
 * Reset an IN endpoint's software TX state.
 *
 * CherryUSB has no direct equivalent of the STM32 USBD_LL_FlushEP() FIFO flush
 * exposed at this layer; the reference uses it to drop any half-queued transfer
 * at (re)initialisation. Here we simply clear the in-use / pending bookkeeping
 * so the next pcan_flush_data()/send_*() is allowed to start a fresh transfer.
 */
int pcan_flush_ep( uint8_t ep )
{
    uint8_t idx = ep & 0x0F;

    pcan_ep_tx_in_use[idx] = 0;
    pcan_ep_tx_data_pending[idx] = 0;
    return 1;
}

/*
 * m2h (device->host) flush FSM, a direct port of the reference pcan_flush_data.
 *
 *   state 0 (idle): copy src into the FSM's dedicated DMA buffer and kick a
 *                   write on pfsm->ep_addr; mark the EP in use; go to state 1.
 *   state 1 (busy): if the EP write has not completed yet, just remember that
 *                   there is pending data (ep_tx_data_pending) and return 0;
 *                   once the IN completion callback cleared the in-use flag,
 *                   fall through to state 0 and send.
 *
 * The caller (pcan_record_buffer_flush in pcan_protocol.c) only resets its
 * record buffer when this returns 1, so "pending" data is naturally retried on
 * the next pcan_protocol_poll(); ep_tx_data_pending is kept for parity with the
 * reference but is otherwise vestigial (the reference's ZLP path is #if 0'd).
 *
 * Returns 1 when a transfer was started, 0 otherwise.
 */
int pcan_flush_data( struct t_m2h_fsm *pfsm, void *src, int size )
{
    uint8_t idx = pfsm->ep_addr & 0x0F;

    switch( pfsm->state )
    {
        case 1:
            if( pcan_ep_tx_in_use[idx] )
            {
                pcan_ep_tx_data_pending[idx] = (uint16_t)size;
                return 0;
            }
            pfsm->state = 0;
            /* fall through */
        case 0:
            if( size > pfsm->dbsize )
                break;
            if( size > 0 )
                memcpy( pfsm->pdbuf, src, (size_t)size );
            pcan_ep_tx_in_use[idx] = 1;
            pcan_ep_tx_data_pending[idx] = 0;
            if( usbd_ep_start_write(PCAN_USB_BUSID, pfsm->ep_addr, pfsm->pdbuf, (uint32_t)size) < 0 )
            {
                pcan_ep_tx_in_use[idx] = 0;
                break;
            }
            pfsm->total_tx += (uint32_t)size;
            pfsm->state = 1;
            return 1;
    }

    return 0;
}

/*
 * Direct "send if the IN endpoint is idle" helpers, a port of the reference
 * pcan_usb_send_command_buffer()/pcan_usb_send_data_buffer(). The reference
 * gates on ep_in[].total_length; here we gate on pcan_ep_tx_in_use[]. The
 * buffer 'p' is DMA'd in place and must stay valid until the IN completion
 * callback fires -- the PCAN-USB host driver is strictly request/response on
 * the command channel, so this matches the reference's aliasing of buffer_cmd.
 * Returns the number of bytes queued, or 0 if the endpoint was busy/failed.
 */
uint16_t pcan_usb_send_command_buffer( const void *p, uint16_t size )
{
    uint8_t idx = PCAN_USB_EP_CMDIN & 0x0F;

    if( pcan_ep_tx_in_use[idx] )
        return 0;

    pcan_ep_tx_in_use[idx] = 1;
    if( usbd_ep_start_write(PCAN_USB_BUSID, PCAN_USB_EP_CMDIN, (const uint8_t *)p, size) < 0 )
    {
        pcan_ep_tx_in_use[idx] = 0;
        return 0;
    }
    return size;
}

uint16_t pcan_usb_send_data_buffer( const void *p, uint16_t size )
{
    uint8_t idx = PCAN_USB_EP_MSGIN & 0x0F;

    if( pcan_ep_tx_in_use[idx] )
        return 0;

    pcan_ep_tx_in_use[idx] = 1;
    if( usbd_ep_start_write(PCAN_USB_BUSID, PCAN_USB_EP_MSGIN, (const uint8_t *)p, size) < 0 )
    {
        pcan_ep_tx_in_use[idx] = 0;
        return 0;
    }
    return size;
}
