#include <stdio.h>
#include <string.h>
#include "board.h"
#include "hpm_mcan_drv.h"
#include "pcan_can.h"
#include "pcan_timestamp.h"

/*
 * Phase 3 CAN driver: HPM5300EVK on-board MCAN3 (Bosch M_CAN IP) behind the
 * frozen pcan_can.h interface, semantically modelled after the reference
 * STM32 bxCAN implementation (pcan_cantact/Src/pcan_can.c).
 *
 * Concurrency model
 * -----------------
 *   - CAN ISR (IRQn_MCAN3): RX drain (RXFIFO0 -> rx_cb) and error/bus-off
 *     reporting only. rx_cb/err_cb are invoked inside a global-IRQ critical
 *     section so the protocol record buffer stays atomic against the USB ISR.
 *   - pcan_can_poll() (main loop AND USB ISR via the protocol's
 *     WAIT_FOR_TXSLOTS loop): TX flush from the 100-deep software ring FIFO
 *     into the MCAN TX FIFO (16 slots) plus deferred bus-off recovery.
 *     The whole dequeue + hardware submit runs inside a critical section, so
 *     the two poll contexts can never double-submit the ring tail.
 *   - Configuration entry points (set_bitrate/set_bus_active/set_silent/
 *     set_loopback, all called from USB ISR context) re-apply mcan_init()
 *     inside a critical section; the hw_started flag is re-checked inside
 *     every critical section that touches the controller.
 *
 * NOTE on echoes: TX-completion echoes are NOT produced here.
 * pcan_protocol.c's pcan_decode_data_frame() already echoes a frame back to
 * the host (with the correct client/echo-id byte) whenever the incoming record
 * carries the SRR (self-receive) bit -- that is how the Linux peak_usb driver
 * requests TX completion. A hardware echo (or setting CAN_FLAG_ECHO on RX)
 * on top of that would deliver duplicate echoes with a bogus echo id (0) and
 * confuse the driver's echo_skb matching.
 */

#define PCAN_CAN_TX_FIFO_SIZE (100)

/* peak_usb programs SJA1000 BTR0/BTR1 assuming an effective clock of
 * PCAN_USB_CRYSTAL_HZ/2 = 8 MHz (see pcan_set_bitrate() in the protocol
 * layer), so host-supplied brp values scale by f_can / 8 MHz. */
#define PCAN_SJA1000_CLK_HZ (8000000UL)

/* MCAN NBTP field limits (CAN 2.0 / nominal): NBRP 1..512, NTSEG1 1..255,
 * NTSEG2 2..128, NSJW 1..128 */
#define PCAN_CAN_PRESCALER_MAX (512U)
#define PCAN_CAN_SEG1_MAX      (255U)
#define PCAN_CAN_SEG2_MIN      (2U)
#define PCAN_CAN_SEG2_MAX      (128U)
#define PCAN_CAN_SJW_MAX       (128U)

#define PCAN_CAN_IRQ_MASK ( MCAN_INT_RXFIFO0_NEW_MSG | MCAN_INT_RXFIFO0_MSG_LOST | \
                            MCAN_INT_TX_COMPLETED | \
                            MCAN_INT_BUS_OFF_STATUS | MCAN_INT_WARNING_STATUS | \
                            MCAN_INT_ERROR_PASSIVE | MCAN_INT_BIT_ERROR_UNCORRECTED | \
                            MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE | \
                            MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE )

/* HPM5300 MCAN message RAM lives in shared AHB SRAM: the buffer must be
 * placed in the .ahb_sram section and handed to the driver via
 * mcan_set_msg_buf_attr() BEFORE mcan_get_default_config() (the default RAM
 * layout is computed from the attribute's offset). */
#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && (MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1)
ATTR_PLACE_AT(".ahb_sram") static uint32_t s_mcan_msg_buf[MCAN_MSG_BUF_SIZE_IN_WORDS];
#endif

static struct
{
  uint32_t tx_msgs;
  uint32_t tx_errs;
  uint32_t tx_ovfs;

  uint32_t rx_msgs;
  uint32_t rx_errs;
  uint32_t rx_ovfs;

  can_message_t tx_fifo[PCAN_CAN_TX_FIFO_SIZE];
  volatile uint32_t tx_head;
  volatile uint32_t tx_tail;

  void (*rx_cb)( can_message_t * );
  void (*can_err_cb)( uint8_t err, uint8_t rx_err, uint8_t tx_err );

  mcan_config_t config;
  uint32_t can_clk_freq;

  /* controller is configured and out of INIT mode */
  volatile uint8_t hw_started;
  /* bus-off seen in ISR, mcan_recover_from_busoff() owed by poll */
  volatile uint8_t bus_off_pending;

  uint8_t bus_active;
  uint8_t silent;
  uint8_t loopback;

  uint16_t brp;
  uint8_t tseg1;
  uint8_t tseg2;
  uint8_t sjw;
}
can_dev = { 0 };

static void pcan_can_isr_handler( MCAN_Type *ptr );
static void pcan_can_flush_tx( void );

static mcan_node_mode_t pcan_can_current_mode( void )
{
  if( can_dev.silent )
    return mcan_mode_listen_only;
  if( can_dev.loopback )
    return mcan_mode_loopback_internal;
  return mcan_mode_normal;
}

/*
 * Map SJA1000 timing (brp/tseg1/tseg2/sjw @ 8 MHz effective clock, as sent by
 * peak_usb) onto MCAN nominal bit timing:
 *
 *   prescaler = brp * (f_can / 8 MHz)            (f_can = 80 MHz -> x10)
 *   num_seg1  = tseg1                            (MCAN num_seg1 excludes the
 *   num_seg2  = tseg2                             sync segment, bit time is
 *   num_sjw   = sjw                               1 + seg1 + seg2 tq, exactly
 *                                                 like SJA1000's 1+tseg1+tseg2)
 *   baudrate  = f_can / (prescaler * (1 + num_seg1 + num_seg2))
 *
 * e.g. 500k: brp=1,tseg1=13,tseg2=2 -> 80M/(10*16) = 500k, SP 87.5%
 *      1M:   brp=1,tseg1=6,tseg2=1  -> seg2 bumped to 2 with seg1=5 (see
 *                                      below), 80M/(10*8) = 1M, SP 75%
 */
static void pcan_can_apply_bitrate( uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw )
{
  mcan_bit_timing_param_t *t = &can_dev.config.can_timing;
  uint32_t freq = can_dev.can_clk_freq;
  uint32_t prescaler;
  uint32_t seg1 = tseg1;
  uint32_t seg2 = tseg2;
  uint32_t sjw_val = sjw;

  if( freq == 0U )
    freq = 80000000UL; /* fallback only; board_init_can_clock() provides the real rate */

  if( ( freq % PCAN_SJA1000_CLK_HZ ) != 0U )
    printf( "[pcan_can] WARN: can clock %u Hz is not a multiple of %u Hz, baudrate will deviate\r\n",
            (unsigned)freq, (unsigned)PCAN_SJA1000_CLK_HZ );

  prescaler = (uint32_t)brp * ( freq / PCAN_SJA1000_CLK_HZ );
  if( prescaler > PCAN_CAN_PRESCALER_MAX )
  {
    printf( "[pcan_can] WARN: prescaler %u exceeds NBRP max %u, clamped (baudrate will deviate)\r\n",
            (unsigned)prescaler, (unsigned)PCAN_CAN_PRESCALER_MAX );
    prescaler = PCAN_CAN_PRESCALER_MAX;
  }

  /* MCAN requires NTSEG2 >= 2 tq while SJA1000 allows tseg2 == 1 (peak_usb
   * does send that, e.g. 1 Mbit with 87.5% sample point). Borrow one tq from
   * seg1 so the total bit time -- and therefore the baudrate -- stays exact;
   * the sample point just moves one tq earlier. */
  if( seg2 < PCAN_CAN_SEG2_MIN )
  {
    if( seg1 >= 2U )
      seg1--;
    else
      printf( "[pcan_can] WARN: degenerate timing tseg1=%u tseg2=%u\r\n",
              (unsigned)tseg1, (unsigned)tseg2 );
    seg2 = PCAN_CAN_SEG2_MIN;
  }
  if( seg1 > PCAN_CAN_SEG1_MAX )
  {
    printf( "[pcan_can] WARN: seg1 %u clamped to %u\r\n", (unsigned)seg1, (unsigned)PCAN_CAN_SEG1_MAX );
    seg1 = PCAN_CAN_SEG1_MAX;
  }
  if( seg2 > PCAN_CAN_SEG2_MAX )
  {
    printf( "[pcan_can] WARN: seg2 %u clamped to %u\r\n", (unsigned)seg2, (unsigned)PCAN_CAN_SEG2_MAX );
    seg2 = PCAN_CAN_SEG2_MAX;
  }
  /* CAN spec: SJW <= TSEG2; SJA1000 sjw is 1..4 so this only matters after the
   * seg2 bump above */
  if( sjw_val > seg2 )
    sjw_val = seg2;
  if( sjw_val > PCAN_CAN_SJW_MAX )
    sjw_val = PCAN_CAN_SJW_MAX;

  t->prescaler  = (uint16_t)prescaler;
  t->num_seg1   = (uint16_t)seg1;
  t->num_seg2   = (uint16_t)seg2;
  t->num_sjw    = (uint8_t)sjw_val;
  t->enable_tdc = false;
}

static void pcan_can_start_hw( void )
{
  hpm_stat_t status;
  uint32_t level;

  can_dev.config.mode = pcan_can_current_mode();

  level = disable_global_irq( CSR_MSTATUS_MIE_MASK );
  status = mcan_init( BOARD_APP_CAN_BASE, &can_dev.config, can_dev.can_clk_freq );
  can_dev.hw_started = ( status == status_success ) ? 1U : 0U;
  can_dev.bus_off_pending = 0U;
  restore_global_irq( level );

  if( status != status_success )
    printf( "[pcan_can] ERROR: mcan_init failed, status=%d\r\n", (int)status );
}

static void pcan_can_stop_hw( void )
{
  uint32_t level = disable_global_irq( CSR_MSTATUS_MIE_MASK );
  can_dev.hw_started = 0U;
  can_dev.bus_off_pending = 0U;
  mcan_deinit( BOARD_APP_CAN_BASE );
  restore_global_irq( level );
}

/*
 * Move at most one frame from the software ring FIFO into the MCAN TX FIFO
 * (reference parity: pcan_can_flush_tx() submits one frame per poll and keeps
 * the ring head queued when the hardware is busy).
 */
static void pcan_can_flush_tx( void )
{
  if( can_dev.tx_head == can_dev.tx_tail )
    return;
  if( !can_dev.hw_started || can_dev.bus_off_pending )
    return;

  uint32_t level = disable_global_irq( CSR_MSTATUS_MIE_MASK );

  /* re-check inside the critical section: poll runs from both the main loop
   * and the USB ISR, and set_* may have stopped/reconfigured the controller
   * in between */
  if( can_dev.hw_started && !can_dev.bus_off_pending &&
      ( can_dev.tx_head != can_dev.tx_tail ) )
  {
    can_message_t *p_msg = &can_dev.tx_fifo[can_dev.tx_tail];
    mcan_tx_frame_t frame;

    memset( &frame, 0, sizeof( frame ) );
    if( p_msg->flags & CAN_FLAG_EXTID )
    {
      frame.use_ext_id = 1U;
      frame.ext_id = p_msg->id & 0x1FFFFFFFUL;
    }
    else
    {
      frame.std_id = p_msg->id & 0x7FFU;
    }
    frame.dlc = ( p_msg->dlc > 8U ) ? 8U : p_msg->dlc; /* classic CAN only */
    if( p_msg->flags & CAN_FLAG_RTR )
    {
      frame.rtr = 1U;
    }
    else
    {
      memcpy( frame.data_8, p_msg->data, frame.dlc );
    }

    if( mcan_transmit_via_txfifo_nonblocking( BOARD_APP_CAN_BASE, &frame, NULL ) == status_success )
    {
      /* submitted to hardware == sent as far as the protocol layer is
       * concerned; on TX FIFO full the head message simply waits for the
       * next poll */
      uint32_t tail = can_dev.tx_tail + 1U;
      if( tail == PCAN_CAN_TX_FIFO_SIZE )
        tail = 0U;
      can_dev.tx_tail = tail;
    }
  }

  restore_global_irq( level );
}

static void pcan_can_isr_handler( MCAN_Type *ptr )
{
  uint32_t flags = mcan_get_interrupt_flags( ptr );
  uint8_t err_bits = 0U;
  uint8_t report_err = 0U;

  /* TX completion: statistics only (reference parity, no echo is produced
   * here -- see the NOTE at the top of this file) */
  if( ( flags & MCAN_INT_TX_COMPLETED ) != 0U )
    can_dev.tx_msgs++;

  if( ( flags & MCAN_INT_RXFIFO0_MSG_LOST ) != 0U )
  {
    can_dev.rx_ovfs++;
    err_bits |= CAN_ERROR_FLAG_RX_OVF;
    report_err = 1U;
  }

  if( ( flags & MCAN_EVENT_ERROR ) != 0U )
  {
    report_err = 1U;
    if( ( flags & MCAN_INT_BUS_OFF_STATUS ) != 0U )
    {
      err_bits |= CAN_ERROR_FLAG_BUSOFF;
      /* recovery runs from pcan_can_poll() to keep the ISR short */
      can_dev.bus_off_pending = 1U;
    }
    if( ( flags & ( MCAN_INT_BIT_ERROR_UNCORRECTED |
                    MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE |
                    MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE ) ) != 0U )
    {
      /* MCAN has no per-mailbox TERR like bxCAN; uncorrected bit errors and
       * arbitration/data phase protocol errors are the transmit-side fault
       * events, so map them onto CAN_ERROR_FLAG_TX_ERR */
      can_dev.tx_errs++;
      err_bits |= CAN_ERROR_FLAG_TX_ERR;
    }
    /* error-warning / error-passive transitions carry no protocol error bit
     * of their own; they still reach the callback (err may be 0) so the
     * host-visible REC/TEC counters stay fresh */
  }

  if( report_err && can_dev.can_err_cb )
  {
    mcan_error_count_t ecnt;
    uint32_t level;

    mcan_get_error_counter( ptr, &ecnt );
    level = disable_global_irq( CSR_MSTATUS_MIE_MASK );
    can_dev.can_err_cb( err_bits, ecnt.receive_error_count, ecnt.transmit_error_count );
    restore_global_irq( level );
  }

  if( ( flags & MCAN_INT_RXFIFO0_NEW_MSG ) != 0U )
  {
    mcan_rx_message_t rx;

    /* drain RXFIFO0 (all filters -- including non-matching frames -- are
     * routed there); bounded by the 32-element FIFO depth */
    while( mcan_read_rxfifo( ptr, 0, &rx ) == status_success )
    {
      can_message_t msg;
      uint32_t level;
      uint8_t dlc;

      memset( &msg, 0, sizeof( msg ) );
      if( rx.use_ext_id )
      {
        msg.id = rx.ext_id;
        msg.flags |= CAN_FLAG_EXTID;
      }
      else
      {
        msg.id = rx.std_id;
      }
      if( rx.rtr )
        msg.flags |= CAN_FLAG_RTR;

      dlc = (uint8_t)rx.dlc;
      if( dlc > 8U )
        dlc = 8U; /* classic CAN: clamp anything longer */
      msg.dlc = dlc;
      if( !rx.rtr )
        memcpy( msg.data, rx.data_8, dlc );

      msg.timestamp = pcan_timestamp_ticks();
      /* CAN_FLAG_ECHO is deliberately NOT set -- TX echoes are generated by
       * the protocol layer's SRR mechanism */

      level = disable_global_irq( CSR_MSTATUS_MIE_MASK );
      if( can_dev.rx_cb )
        can_dev.rx_cb( &msg );
      restore_global_irq( level );

      can_dev.rx_msgs++;
    }
  }

  mcan_clear_interrupt_flags( ptr, flags );
}

SDK_DECLARE_EXT_ISR_M( BOARD_APP_CAN_IRQn, pcan_can_hw_isr )
void pcan_can_hw_isr( void )
{
  pcan_can_isr_handler( BOARD_APP_CAN_BASE );
}

void pcan_can_init( void )
{
  MCAN_Type *base = BOARD_APP_CAN_BASE;
  hpm_stat_t status;

#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && (MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1)
  {
    mcan_msg_buf_attr_t attr;
    attr.ram_base = (uint32_t)&s_mcan_msg_buf[0];
    attr.ram_size = sizeof( s_mcan_msg_buf );
    if( mcan_set_msg_buf_attr( base, &attr ) != status_success )
      printf( "[pcan_can] ERROR: invalid MCAN message buffer attribute\r\n" );
  }
#endif

  board_init_can( base ); /* PY04 MCAN3_RXD / PY05 MCAN3_TXD */
  can_dev.can_clk_freq = board_init_can_clock( base );
  printf( "[pcan_can] MCAN clock = %u Hz (expected %u Hz)\r\n",
          (unsigned)can_dev.can_clk_freq, 80000000U );

  mcan_get_default_config( base, &can_dev.config );
  can_dev.config.use_lowlevel_timing_setting = true;
  can_dev.config.enable_canfd = false;                /* classic CAN 2.0 only */
  can_dev.config.disable_auto_retransmission = false; /* automatic retransmission on */
  can_dev.config.interrupt_mask = PCAN_CAN_IRQ_MASK;
  can_dev.config.txbuf_trans_interrupt_mask = ~0UL;
  /* default filter setup already accepts everything: one accept-all classic
   * filter each for standard and extended IDs routed to RXFIFO0, non-matching
   * frames also go to RXFIFO0, remote frames are not rejected */

  /* sane 500 kbit/s default until the host programs the real bitrate */
  can_dev.brp   = 1U;
  can_dev.tseg1 = 13U;
  can_dev.tseg2 = 2U;
  can_dev.sjw   = 1U;
  pcan_can_apply_bitrate( can_dev.brp, can_dev.tseg1, can_dev.tseg2, can_dev.sjw );
  can_dev.config.mode = mcan_mode_normal;

  /* validate the full configuration once, then stop the controller again:
   * the bus only goes up when the host asks for it (set_bus_active), which
   * mirrors the reference HAL_CAN_Init()/HAL_CAN_Start() split */
  status = mcan_init( base, &can_dev.config, can_dev.can_clk_freq );
  if( status != status_success )
    printf( "[pcan_can] ERROR: initial mcan_init failed, status=%d\r\n", (int)status );
  mcan_deinit( base );
  can_dev.hw_started = 0U;

  intc_m_enable_irq_with_priority( BOARD_APP_CAN_IRQn, 1 );
}

void pcan_can_poll( void )
{
  if( !can_dev.hw_started )
    return;

  /* deferred bus-off recovery (reported from the ISR): re-join the bus, the
   * M_CAN then waits for 128 x 11 recessive bits like bxCAN AutoBusOff */
  if( can_dev.bus_off_pending )
  {
    uint32_t level = disable_global_irq( CSR_MSTATUS_MIE_MASK );
    if( can_dev.bus_off_pending && can_dev.hw_started )
    {
      /* on failure keep the flag and retry on the next poll */
      if( mcan_recover_from_busoff( BOARD_APP_CAN_BASE ) == status_success )
        can_dev.bus_off_pending = 0U;
    }
    restore_global_irq( level );
  }

  pcan_can_flush_tx();
}

void pcan_can_install_rx_callback( void (*cb)( can_message_t * ) )
{
  can_dev.rx_cb = cb;
}

void pcan_can_install_error_callback( void (*cb)( uint8_t, uint8_t, uint8_t ) )
{
  can_dev.can_err_cb = cb;
}

void pcan_can_set_bitrate( uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw )
{
  can_dev.brp   = brp;
  can_dev.tseg1 = tseg1;
  can_dev.tseg2 = tseg2;
  can_dev.sjw   = sjw;

  pcan_can_apply_bitrate( brp, tseg1, tseg2, sjw );

  printf( "[pcan_can] set_bitrate brp=%u tseg1=%u tseg2=%u sjw=%u -> presc=%u seg1=%u seg2=%u sjw=%u\r\n",
          (unsigned)brp, (unsigned)tseg1, (unsigned)tseg2, (unsigned)sjw,
          (unsigned)can_dev.config.can_timing.prescaler,
          (unsigned)can_dev.config.can_timing.num_seg1,
          (unsigned)can_dev.config.can_timing.num_seg2,
          (unsigned)can_dev.config.can_timing.num_sjw );

  /* keep current bus_active/silent/loopback state; re-init only when running */
  if( can_dev.hw_started )
    pcan_can_start_hw();
}

int pcan_can_send_message( const can_message_t *p_msg )
{
  uint32_t head;

  if( !p_msg )
    return 0;

  head = can_dev.tx_head + 1U;
  if( head == PCAN_CAN_TX_FIFO_SIZE )
    head = 0U;
  /* overflow ? just skip it (the protocol layer then sets TXFULL) */
  if( head == can_dev.tx_tail )
  {
    ++can_dev.tx_ovfs;
    return -1;
  }

  can_dev.tx_fifo[can_dev.tx_head] = *p_msg;
  can_dev.tx_head = head;

  return 0;
}

void pcan_can_set_bus_active( uint16_t mode )
{
  printf( "[pcan_can] set_bus_active mode=%u\r\n", (unsigned)mode );

  if( mode )
  {
    can_dev.bus_active = 1U;
    pcan_can_start_hw();
  }
  else
  {
    can_dev.bus_active = 0U;
    pcan_can_stop_hw();
  }
}

void pcan_can_set_silent( uint8_t silent_mode )
{
  can_dev.silent = silent_mode ? 1U : 0U;
  printf( "[pcan_can] set_silent=%u\r\n", (unsigned)can_dev.silent );

  if( can_dev.hw_started )
    pcan_can_start_hw();
}

void pcan_can_set_loopback( uint8_t loopback )
{
  can_dev.loopback = loopback ? 1U : 0U;
  printf( "[pcan_can] set_loopback=%u\r\n", (unsigned)can_dev.loopback );

  if( can_dev.hw_started )
    pcan_can_start_hw();
}
