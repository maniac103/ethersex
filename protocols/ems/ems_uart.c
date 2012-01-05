#include <avr/interrupt.h>
#include "config.h"
#include "ems.h"

#define BAUD 9600

static uint8_t has_pending_tx = 0;

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
  if (ems_send_buffer.sent < ems_send_buffer.len) {
    usart(UDR) = ems_send_buffer.data[ems_send_buffer.sent++];
  } else {
    /* Disable this interrupt */
    usart(UCSR,B) &= ~(_BV(usart(TXCIE)));
    usart(UCSR,B) |= _BV(usart(RXCIE));
  }
}

ISR(usart(USART,_RX_vect))
{
  uint8_t status;

  while ((status = (usart(UCSR,A) & _BV(usart(RXC))))) {
    uint8_t data = usart(UDR);
    ems_uart_process_input_byte(data, status);
  }

  if (has_pending_tx) {
    usart(UCSR,B) &= ~_BV(usart(RXCIE));
    usart(UCSR,B) |= _BV(usart(TXCIE));
  }
}

#else /* !EMS_SOFT_UART */

#define PRESCALE 8
#define BIT_TIME  ((uint8_t)((F_CPU / BAUD) / PRESCALE))

static volatile uint8_t current_rx_mask;
static volatile uint8_t current_rx_data;

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

  current_rx_mask = 1;
  current_rx_data = 0;

  if (!PIN_HIGH(EMS_UART_RX)) {
    set_rx_int(0);
    TC2_INT_COMPARE_ON;
  }
}

ISR(TC2_VECTOR_COMPARE)
{
  uint8_t in = PIN_HIGH(EMS_UART_RX);
  if (current_rx_mask) {
    if (in) {
      current_rx_data |= current_rx_mask;
    }
    current_rx_mask <<= 1;
    TC2_COUNTER_COMPARE += BIT_TIME;
  } else {
    uint8_t status = in ? 0 : _BV(FE);
    ems_uart_process_input_byte(current_rx_data, status);
    if (has_pending_tx) {
      /* start TX */
    } else {
      TC2_INT_COMPARE_OFF;
      set_rx_int(1);
    }
  }
}

#endif /* !EMS_SOFT_UART */

#if 0

volatile u8 stx_count;
u8 stx_data;

volatile u8 srx_done;
u8 srx_data;
u8 srx_mask;
u8 srx_tmp;


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


void suart_init( void )
{
	SBIT(PORTD,STX) = 1;
    TCCR0B = 1 << CS01 | 1<<CS00;   // clk/64
		    TIMSK0 = 1 << OCIE0A;	// enable output compare interrupt

		    EICRA = 1 << ISC01;	    // falling edge
	    EIMSK = 1 << INT0;		// enable edge interrupt

	    			stx_count = 0;		    // nothing to sent
	    				    srx_done = 0;	        // nothing received
	    					STXDDR |= 1 << STX;	    // TX output
}

ISR (INT0_vect)    // rx start
{
    OCR0B = TCNT0 + (u8)((BIT_TIME * 3) / 2);// scan 1.5 bits after start

    srx_tmp = 0;        // clear bit storage
    srx_mask = 1;        // bit mask
    if( !(SRXPIN & 1<<SRX))  {  // still low
	EIMSK &= ~(1 << INT0);	    // disable edge interrupt
	TIMSK0 = 1<<OCIE0A^1<<OCIE0B;  // wait for first bit
    }
    TIFR0 = 1<<OCF0B;      // clear pending interrupt ? why does that output compare int occur?
    EIFR |= (1 << INTF0);	    // clear any pending edge interrupt
}

ISR (TIMER0_COMPB_vect)
{
    u8 in = SRXPIN;      // scan rx line
    if (srx_mask) {
	if (in & 1 << SRX)
	    srx_tmp |= srx_mask;
	srx_mask <<= 1;
	OCR0B += BIT_TIME;      // next bit slice
    } else {
	srx_data = srx_tmp;      // store rx data
	TIMSK0 = 1<<OCIE0A;        // enable tx and wait for start
	EIFR |= (1 << INTF0);	    // clear any pending edge interrupt: This hinders the in0-vect from beeing triggerd again just now which may occur by falling edges in the serial data bits
	EIMSK = 1 << INT0;  // reenable edge interrupt
    }
}

ISR (TIMER0_COMPA_vect)    // tx bit
{
    u8 dout;
    u8 count;

    OCR0A += BIT_TIME;      // next bit slice
    count = stx_count;

    if (count) {
	stx_count = --count;    // count down
	dout = 0;
	if (count != 9) {      // no start bit
	    if (!(stx_data & 1))    // test inverted data
		dout = 1;
	    stx_data >>= 1;      // shift zero in from left
	}
	SBIT(PORTD,STX) = dout;
    }
}


#endif
