/*
 * File:         include/asm-blackfin/bfin_sdh.h
 * Based on:
 * Author:       Cliff Cai (cliff.cai@analog.com)
 *
 * Created:      Mon Dec. 29 2008
 * Description:
 *
 * Modified:
 *               Copyright 2008-2010 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
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
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __BFIN_SDH_H__
#define __BFIN_SDH_H__

struct bfin_sd_host {
	int dma_chan;
	int irq_int0;
	int irq_int1;
	u16 pin_req[6];
};

#endif
