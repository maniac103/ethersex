/*
 *
 * Copyright (c) 2008 by Christian Dietrich <stettberger@dokucode.de>
 * Copyright (c) 2010 by Erik Kunze <ethersex@erik-kunze.de>
 * Copyright (c) 2011 by Danny Baumann <dannybaumann@web.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * For more information on the GPL, please go to:
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <avr/interrupt.h>
#include <string.h>
#include "config.h"
#include "ems.h"
#include "ems_net.h"

struct ems_uart_input_buffer ems_input_buffer;
struct ems_buffer ems_send_buffer;
struct ems_buffer ems_recv_buffer;
#ifdef DEBUG_EMS
struct ems_stats ems_stats_buffer;
#endif

void
ems_init(void)
{
  ems_uart_init();
}

uint8_t
ems_process_txdata(uint8_t *data, uint16_t len)
{
  uint16_t diff = ems_send_buffer.len - ems_send_buffer.sent;
  if (diff == 0) {
    /* Copy the data to the send buffer */
    memcpy(ems_send_buffer.data, data, len);
    ems_send_buffer.len = len;
    /* The actual packet can be pushed into the buffer */
  } else if ((diff + len) < EMS_BUFFER_LEN) {
    memmove(ems_send_buffer.data, ems_send_buffer.data + ems_send_buffer.sent, diff);
    memcpy(ems_send_buffer.data + diff, data, len);
    ems_send_buffer.len = diff + len;
  } else {
    return 0;
  }

  return 1;
}

void
ems_uart_process_input_byte(uint8_t data, uint8_t status)
{
  uint8_t index = ems_input_buffer.count;

  if (status & _BV(FE)) {
    /* start-of-frame */
    ems_input_buffer.data[index].data = 0;
    ems_input_buffer.data[index].control = 1; // XXX
    ems_input_buffer.count++;
  } else if (status & (_BV(DOR) | _BV(UPE))) {
    /* error -> drop */
  } else {
    ems_input_buffer.data[index].data = data;
    ems_input_buffer.data[index].control = 0;
    ems_input_buffer.count++;
  }

  if (ems_input_buffer.count >= EMS_UART_INPUT_BUFSIZE) {
    UPDATE_STATS(buffer_overflow, 1);
    ems_input_buffer.count = 0;
  }
}

/*
  -- Ethersex META --
  dnl ems_init call must be done after network_init (according to earlier
  dnl comments.  Therefore we initialize via net_init and control the
  dnl order via the Makefile.
  
  header(protocols/ems/ems.h)

  net_init(ems_init)
*/