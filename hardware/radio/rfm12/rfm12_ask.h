/*
 * Copyright (c) Gregor B.
 * Copyright (c) 2009 Dirk Pannenbecker <dp@sd-gp.de>
 * Copyright (c) 2012-14 by Erik Kunze <ethersex@erik-kunze.de>
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

#ifndef __RFM12_ASK_H
#define __RFM12_ASK_H

#include <stdint.h>

#include "hardware/radio/rfm12/rfm12.h"


#define ASK_TX_ENABLE \
	rfm12_prologue(RFM12_MODULE_ASK); \
	rfm12_trans(RFM12_CMD_PWRMGT | RFM12_PWRMGT_ET | RFM12_PWRMGT_ES | RFM12_PWRMGT_EX);
#define ASK_TX_DISABLE \
	rfm12_trans(RFM12_CMD_PWRMGT | RFM12_PWRMGT_EX); \
	rfm12_epilogue();
#define ASK_TX_TRIGGER \
	rfm12_ask_trigger

void rfm12_ask_trigger(uint8_t, uint16_t);
void rfm12_ask_external_filter_init(void);
void rfm12_ask_external_filter_deinit(void);
void rfm12_ask_init(void);

#endif /* __RFM12_ASK_H */
