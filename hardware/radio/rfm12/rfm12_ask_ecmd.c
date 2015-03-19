/*
 * Copyright (c) 2009 Dirk Pannenbecker <dp@sd-gp.de>
 * Copyright (c) Gregor B.
 * Copyright (c) Dirk Pannenbecker
 * Copyright (c) Guido Pannenbecker
 * Copyright (c) Stefan Riepenhausen
 * Copyright (c) 2012-14 Erik Kunze <ethersex@erik-kunze.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 *
 * For more information on the GPL, please go to:
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <stdio.h>
#include <avr/pgmspace.h>

#include "config.h"
#include "core/debug.h"

#include "rfm12.h"
#include "rfm12_ask.h"
#include "rfm12_ask_sense.h"

#include "protocols/ecmd/ecmd-base.h"


#ifdef RFM12_ASK_EXTERNAL_FILTER_SUPPORT
int16_t
parse_cmd_rfm12_ask_external_filter(char *cmd, char *output, uint16_t len)
{
  (void) output;
  (void) len;

  uint8_t flag;
  int ret = sscanf_P(cmd, PSTR("%hhu"), &flag);
  if (ret == 1 && flag == 1)
    rfm12_ask_external_filter_init();
  else
    rfm12_ask_external_filter_deinit();

  return ECMD_FINAL_OK;
}

#ifdef RFM12_ASK_SENSING_SUPPORT
int16_t
parse_cmd_rfm12_ask_sense(char *cmd, char *output, uint16_t len)
{
  (void) cmd;
  (void) output;
  (void) len;

  rfm12_ask_sense_start();
  return ECMD_FINAL_OK;
}
#endif /* RFM12_ASK_SENSING_SUPPORT */
#endif /* RFM12_ASK_EXTERNAL_FILTER_SUPPORT */

/*
  -- Ethersex META --
  block([[RFM12_ASK]])
  ecmd_ifdef(RFM12_ASK_EXTERNAL_FILTER_SUPPORT)
    ecmd_feature(rfm12_ask_external_filter, "rfm12 external filter",[1], Enable ext. filter pin if argument is present (disable otherwise))
  ecmd_endif()
  ecmd_ifdef(RFM12_ASK_SENSING_SUPPORT)
    ecmd_feature(rfm12_ask_sense, "rfm12 ask sense",, Trigger (Tevion) ASK sensing.  Enable ext. filter pin before!)
  ecmd_endif()
*/
