#include <stdio.h>
#include "board.h"
#include "usb_config.h"
#include "pcan_timestamp.h"
#include "pcan_led.h"
#include "pcan_usb.h"

int main(void)
{
    board_init();
    board_init_usb((USB_Type *)CONFIG_HPM_USBD_BASE);

    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 1);

    pcan_timestamp_init();
    pcan_led_init();
    pcan_usb_init();

    printf("pcan_hpm5300 phase1 usb enumeration\n");

    while (1) {
        pcan_led_poll();
    }
    return 0;
}
