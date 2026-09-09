#include "board.h"
#include "hpm_gpio_drv.h"
#include "pcan_timestamp.h"
#include "pcan_led.h"

/*
 * hpm5300evk has a single user LED on PA23 (HPM_GPIO0 / GPIO_DI_GPIOA / pin 23),
 * active-low: ON = drive 0, OFF = drive 1. LED_CH0_TX and LED_CH0_RX are both
 * mapped onto this one pin (their logical states are OR'd before writing), and
 * LED_STAT has no dedicated pin so it is ignored.
 */
#define PCAN_LED_CTRL   BOARD_LED_GPIO_CTRL
#define PCAN_LED_INDEX  BOARD_LED_GPIO_INDEX
#define PCAN_LED_PIN    BOARD_LED_GPIO_PIN
#define PCAN_LED_ON     BOARD_LED_ON_LEVEL   /* 0 */
#define PCAN_LED_OFF    BOARD_LED_OFF_LEVEL  /* 1 */

static struct
{
  uint16_t mode;
  uint16_t arg;
  uint16_t delay;
  uint16_t timestamp;
  uint8_t  state;
}
led_mode_array[LED_TOTAL] = { 0 };

void pcan_led_init( void )
{
  clock_add_to_group( clock_gpio, 0 );
  /* mux PA23 -> GPIO_A_23, set as output and drive to OFF level */
  board_init_led_pins();
}

void pcan_led_set_mode( int led, int mode, uint16_t arg )
{
  if( led >= LED_TOTAL )
    return;
  uint16_t ts = pcan_timestamp_millis();

  led_mode_array[led].mode = mode;
  if( !led_mode_array[led].timestamp )
  {
    led_mode_array[led].timestamp = ts|1;
  }
  led_mode_array[led].delay = 0;

  /* set guard time */
  if( mode == LED_MODE_BLINK_FAST || mode == LED_MODE_BLINK_SLOW )
  {
    led_mode_array[led].delay = ( mode == LED_MODE_BLINK_FAST ) ? 50: 200;
    arg = arg?(ts + arg)|1:0;
  }

  led_mode_array[led].arg  = arg;
}

static void pcan_led_update_state( int led, uint8_t state )
{
  (void)led;
  (void)state;
  /* OR the TX and RX logical states onto the single physical LED */
  uint8_t on = ( led_mode_array[LED_CH0_TX].state || led_mode_array[LED_CH0_RX].state );
  gpio_write_pin( PCAN_LED_CTRL, PCAN_LED_INDEX, PCAN_LED_PIN, on ? PCAN_LED_ON : PCAN_LED_OFF );
}

void pcan_led_poll( void )
{
  uint16_t ts_ms = pcan_timestamp_millis();

  for( int i = 0; i < LED_TOTAL; i++ )
  {
    if( !led_mode_array[i].timestamp  )
      continue;
    if( (uint16_t)( ts_ms - led_mode_array[i].timestamp ) < led_mode_array[i].delay )
      continue;

    switch( led_mode_array[i].mode )
    {
      default:
      case LED_MODE_NONE:
        led_mode_array[i].timestamp = 0;
      break;
      case LED_MODE_OFF:
      case LED_MODE_ON:
        led_mode_array[i].state = ( led_mode_array[i].mode == LED_MODE_ON );
        led_mode_array[i].timestamp = 0;
      break;
      case LED_MODE_BLINK_FAST:
      case LED_MODE_BLINK_SLOW:
        led_mode_array[i].state ^= 1;
        led_mode_array[i].timestamp += led_mode_array[i].delay;
        led_mode_array[i].timestamp |= 1;
        if( led_mode_array[i].arg && ( led_mode_array[i].arg <= ts_ms ) )
        {
          pcan_led_set_mode( i, LED_MODE_OFF, 0 );
        }
      break;
    }

    pcan_led_update_state( i, led_mode_array[i].state );
  }
}
