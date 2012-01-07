#include <avr/interrupt.h>
#include "config.h"
#include "ems.h"

#define BAUD 9600

static uint8_t has_pending_tx(void)
{
  return ems_send_buffer.sent < ems_send_buffer.len;
}

static uint8_t next_tx_byte(void)
{
  if (has_pending_tx()) {
    return ems_send_buffer.data[ems_send_buffer.sent++];
  }
  return 0;
}

#if !defined(EMS_SOFT_UART)
#define USE_USART EMS_USE_USART
#include "core/usart.h"
/* We generate our own usart init module, for our usart port */
generate_usart_init()

void
ems_uart_init(void)
{
  usart_init();
}

ISR(usart(USART,_TX_vect))
{
  if (has_pending_tx()) {
    usart(UDR) = next_tx_byte();
  } else {
    /* Disable this interrupt */
    usart(UCSR,B) &= ~(_BV(usart(TXCIE)));
    usart(UCSR,B) |= _BV(usart(RXCIE));
    ems_set_led(LED_BLUE, 0, 0);
  }
}

ISR(usart(USART,_RX_vect))
{
  uint8_t status;

  while ((status = (usart(UCSR,A) & _BV(usart(RXC))))) {
    uint8_t data = usart(UDR);
    ems_uart_process_input_byte(data, status);
  }

  if (has_pending_tx()) {
    ems_set_led(LED_BLUE, 1, 0);
    usart(UCSR,B) &= ~_BV(usart(RXCIE));
    usart(UCSR,B) |= _BV(usart(TXCIE));
  }
}

#else /* !EMS_SOFT_UART */

#define PRESCALE 8
#define BIT_TIME  ((uint8_t)((F_CPU / BAUD) / PRESCALE))

static volatile uint8_t current_mask;
static volatile uint8_t current_data;
static volatile uint8_t sending = 0;
static volatile uint8_t send_counter = 0;

static void set_rx_int(uint8_t enable)
{
  if (enable) {
    GIFR = _BV(EMS_SOFTRX_INT_PIN);
    _EIMSK |= _BV(EMS_SOFTRX_INT_PIN);
  } else {
    _EIMSK &= ~_BV(EMS_SOFTRX_INT_PIN);
  }
}
void
ems_uart_init(void)
{
  DDR_CONFIG_IN(EMS_UART_RX);
  DDR_CONFIG_OUT(EMS_UART_TX);
  PIN_SET(EMS_UART_TX);

  TC2_COUNTER_CURRENT = 0;
  TC2_PRESCALER_8;

  /* Initialize Interrupt */
  _EICRA = (uint8_t) ((_EICRA & ~EMS_SOFTRX_INT_ISCMASK) | EMS_SOFTRX_INT_ISC);
  set_rx_int(1);
}

ISR(EMS_SOFTRX_INT_VECTOR)
{
  TC2_COUNTER_COMPARE = TC2_COUNTER_CURRENT + (uint8_t)((BIT_TIME * 3) / 2);// scan 1.5 bits after start

  current_mask = 1;
  current_data = 0;

  if (!PIN_HIGH(EMS_UART_RX)) {
    set_rx_int(0);
    TC2_INT_COMPARE_ON;
  }
}

static void
check_tx_or_go_to_rx(void)
{
  if (has_pending_tx()) {
    ems_set_led(LED_BLUE, 1, 0);
    sending = 1;
    current_data = next_tx_byte();
    send_counter = 9;
    TC2_OUTPUT_COMPARE_SET;
  } else {
    ems_set_led(LED_BLUE, 0, 0);
    sending = 0;
    TC2_OUTPUT_COMPARE_NONE;
    TC2_INT_COMPARE_OFF;
    set_rx_int(1);
  }
}

ISR(TC2_VECTOR_COMPARE)
{
  uint8_t in = PIN_HIGH(EMS_UART_RX);
  TC2_COUNTER_COMPARE += BIT_TIME;

  if (sending) {
    if (send_counter == 0) {
      /* byte including stop is done */
      check_tx_or_go_to_rx();
    } else {
      send_counter--;

      uint8_t lastbit = send_counter == 8 ? 0 : (current_data & _BV(send_counter));
      uint8_t nextbit = send_counter == 0 ? 1 : (current_data & _BV(send_counter - 1)); // need FE?

      if ((lastbit && !in) || (!lastbit && in)) {
        /* abort */
      }

      if (nextbit) {
        TC2_OUTPUT_COMPARE_SET;
      } else {
        TC2_OUTPUT_COMPARE_CLEAR;
      }
    }
  } else /* receiving */ {
    if (current_mask) {
      if (in) {
        current_data |= current_mask;
      }
      current_mask <<= 1;
    } else {
      uint8_t status = in ? 0 : _BV(FE);
      ems_uart_process_input_byte(current_data, status);
      check_tx_or_go_to_rx();
    }
  }
}

#endif /* !EMS_SOFT_UART */

#if 0

/* send:
 */
dr = rx_data;

/* sending */
if ( ems_flag & 1 )
{
    if ( dr == 0x0b )
    {
        /* send uart break */
    }
    ems_flag &= ~1;
}

if ( ems_cnt == 0 )
    ems_adr = dr;
if ( (ems_cnt == 1) && (dr!=0) && (ems_adr&0x80) )
{
    ems_cnt = 0;
    ems_adr = dr;
}
/* ignore frame error */
if ( !(sr & (UART_SR_FE)) )
{
    ems_cnt++;
} else {
    if ( ems_adr == 0x8b )
    {
        tx_data = 0x0b;
        ems_flag |= 1;
    }
}

#endif
