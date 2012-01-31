#include <avr/interrupt.h>
#include "config.h"
#include "ems.h"

#define BAUD 9600

/*
 * theory:
 * Polling: address | 0x80
 * Answer:
 * 0) nothing to send: <addr> <break>
 * 1) Broadcast: <addr> 0x0 <data> ... <break>
 * 2) Send without request: <addr> <dest> <data> ... <crc> <break>
 * 3) Send with request: <addr> <dest | 0x80> <data> ... <crc> <break>
 *
 * -> send <addr> <txdata> <break>
 *
 * after tx byte compare with rx
 * if mismatch -> abort with <break>
 */

/* test data to send: "88 02 00 06" */

#define STATE_RX                 0
#define STATE_TX_ADDR            1
#define STATE_TX_ADDR_WAIT_ECHO  2
#define STATE_TX_DATA            3
#define STATE_TX_DATA_WAIT_ECHO  4
#define STATE_TX_BREAK           5

#define PRESCALE 8
#define BIT_TIME  ((uint8_t)((F_CPU / BAUD) / PRESCALE))
#define TX_TIMEOUT 10 /* x100ms = 1 second */

#define USE_USART 1
#include "core/usart.h"

static volatile uint8_t state = STATE_RX;
static volatile uint8_t bit_counter = 0;
static volatile uint8_t tx_timeout = 0;

/* We generate our own usart init module, for our usart port */
generate_usart_init()

void
ems_uart_init(void)
{
  usart_init();

  PIN_CLEAR(EMS_UART_TX);
  DDR_CONFIG_OUT(EMS_UART_TX);

  TC2_PRESCALER_8;
  TC2_MODE_CTC;
}

static inline uint8_t is_polled(void)
{
  uint8_t polled = (ems_poll_address == (OUR_EMS_ADDRESS | 0x80));
  ems_poll_address = 0;
  return polled;
}

static uint8_t get_next_tx_byte(uint8_t *byte)
{
  if (ems_send_buffer.sent < ems_send_buffer.len) {
    *byte = ems_send_buffer.data[ems_send_buffer.sent++];
    return 1;
  }
  return 0;
}

static void
switch_mode(uint8_t tx)
{
  uint8_t reg = usart(UCSR, B);
  if (tx) {
    reg &= ~(_BV(usart(RXEN)) | _BV(usart(RXCIE)));
    reg |= _BV(usart(UDRIE));
  } else {
    reg &= ~(_BV(usart(UDRIE)));
    reg |= _BV(usart(RXEN)) | _BV(usart(RXCIE));
  }
  usart(UCSR, B) = reg;
}

static void
go_to_rx(void)
{
  ems_set_led(LED_BLUE, 1, 1);
  /* drain input buffer */
  while (usart(UCSR,A) & _BV(usart(RXC))) {
    uint8_t data = usart(UDR);
  }
  switch_mode(0);
  state = STATE_RX;
}

void
ems_uart_periodic(void)
{
  if (tx_timeout == 0) {
    return;
  }

  tx_timeout--;
  if (tx_timeout == 0) {
    go_to_rx();
  }
}

ISR(TC2_VECTOR_COMPARE)
{
  bit_counter--;
  if (bit_counter == 0) {
    TC2_INT_COMPARE_OFF;
    usart(UCSR,B) |= _BV(usart(TXEN));
    tx_timeout = 0;
    go_to_rx();
  }
}

ISR(usart(USART, _TX_vect))
{
  /* TX finished, now send break */
  usart(UCSR,B) &= ~(_BV(usart(UDRIE)) | _BV(usart(TXEN)) | _BV(usart(TXCIE)));
  bit_counter = 11;
  TC2_COUNTER_COMPARE = BIT_TIME;
  TC2_COUNTER_CURRENT = 0;
  TC2_INT_COMPARE_ON;
  state = STATE_TX_BREAK;
}

ISR(usart(USART,_UDRE_vect))
{
  switch (state) {
    case STATE_TX_ADDR:
      usart(UDR) = OUR_EMS_ADDRESS;
      state = STATE_TX_ADDR_WAIT_ECHO;
      tx_timeout = TX_TIMEOUT;
      switch_mode(0);
      break;
    case STATE_TX_DATA:
      {
        uint8_t byte;
        if (get_next_tx_byte(&byte)) {
          usart(UDR) = byte;
          state = STATE_TX_DATA_WAIT_ECHO;
          tx_timeout = TX_TIMEOUT;
          switch_mode(0);
        } else {
          /* wait for TX to finish */
          usart(UCSR,B) &= ~(_BV(usart(UDRIE)));
          usart(UCSR,B) |= _BV(usart(TXCIE));
        }
      }
      break;
    case STATE_TX_BREAK:
    default:
      /* Disable this interrupt */
      usart(UCSR,B) &= ~(_BV(usart(UDRIE)));
      go_to_rx();
      break;
  }
}

ISR(usart(USART,_RX_vect))
{
  uint8_t status;

  switch (state) {
    case STATE_TX_ADDR_WAIT_ECHO:
    case STATE_TX_DATA_WAIT_ECHO:
      state = STATE_TX_DATA;
      tx_timeout = 0;
      switch_mode(1);
      break;
    default:
      while ((status = usart(UCSR,A)) & _BV(usart(RXC))) {
        uint8_t data = usart(UDR);
        uint8_t real_status = 0;

        if (status & _BV(usart(FE))) real_status |= FRAMEEND;
        if (status & (_BV(usart(DOR)) | _BV(usart(UPE)))) real_status |= ERROR;

        ems_uart_process_input_byte(data, real_status);
      }

      if (is_polled()) {
        UPDATE_STATS(onebyte_own_packets, 1);
        ems_set_led(LED_BLUE, 1, 0);
        state = STATE_TX_ADDR;
        switch_mode(1);
#ifdef EMS_DEBUG
        if (ems_send_buffer.sent != ems_send_buffer.len) {
          EMSDEBUG("Sending %d bytes\n", ems_send_buffer.len - ems_send_buffer.sent);
        }
#endif
      }
      break;
  }
}

/*
  -- Ethersex META --
  timer(5, ems_uart_periodic())
*/
