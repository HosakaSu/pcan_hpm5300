#include "board.h"
#include "hpm_mchtmr_drv.h"
#include "pcan_timestamp.h"

/*
 * mchtmr free-runs at 24 MHz (clock_mchtmr0 = osc24m / 1, set by board_init).
 * A PCAN tick is 42.666us == 1024 counts @24MHz, so ticks = count >> 10.
 */

void pcan_timestamp_init( void )
{
  /* mchtmr is already running after board_init(); nothing to configure. */
}

uint16_t pcan_timestamp_ticks( void )
{
  /* 1 pcan tick => 42.666us => 1024 counts @24MHz */
  return (uint16_t)( mchtmr_get_count( HPM_MCHTMR ) >> 10 );
}

uint16_t pcan_timestamp_millis( void )
{
  return (uint16_t)( mchtmr_get_count( HPM_MCHTMR ) / 24000u );
}
