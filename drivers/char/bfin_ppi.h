/*
 * File:         drivers/char/bfin_ppi.h
 * Based on:
 * Author:       John DeHority <john.dehority@NOSPAM@kodak.com>
 *
 * Created:      May 5, 2005
 * Description:  It's driver of PPI in Analog Devices BF533 DSP
 *
 * Modified:
 *               Copyright (C) 2005 Eastman Kodak Company
 *               Copyright 2005-2009 Analog Devices Inc.
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

#ifndef __BFIN_PPI_H__
#define __BFIN_PPI_H__

/*
** ioctl commands
*/
#define CMD_PPI_XFR_TYPE       3
#define CMD_PPI_PORT_CFG       4
#define CMD_PPI_FIELD_SELECT   5
#define CMD_PPI_PACKING        6
#define CMD_PPI_SKIPPING       7
#define CMD_PPI_SKIP_ODDEVEN   8
#define CMD_PPI_DATALEN        9
#define	CMD_PPI_CLK_EDGE      10
#define CMD_PPI_TRIG_EDGE     11
#define CMD_PPI_LINELEN		  12
#define CMD_PPI_NUMLINES      13
#define CMD_PPI_SET_WRITECONTINUOUS 14
#define CMD_PPI_SET_DIMS	  15
#define CMD_PPI_DELAY	  	  16
#define	CMD_PPI_SETGPIO		  17
#define	CMD_PPI_GEN_FS12_TIMING_ON_WRITE	18

#define CMD_PPI_FSACTIVE_SELECT	19
#define CMD_PPI_BLANKGEN_SELECT 20
#define CMD_PPI_CLKGEN_SELECT 21
#define CMD_PPI_FSGEN_SELECT 22
#define CMD_PPI_SWAP 23
#define CMD_PPI_SIGNEXT 24
#define CMD_PPI_SPLIT_EVENODD 25
#define CMD_PPI_SUBSPLIT_ODD 26
#define CMD_PPI_RGBFORMAT 27
#define CMD_PPI_FIFORWM 28
#define CMD_PPI_FIFOUWM 29
#define CMD_PPI_ITUTYPE 30
#define CMD_PPI_CLKDIV 31
#define CMD_PPI_EPPI1_FS1W_HBL 32
#define CMD_PPI_EPPI1_FS1P_AVPL 33
#define CMD_PPI_EPPI1_FS2W_LVB 34
#define CMD_PPI_EPPI1_FS2P_LAVF 35
#define CMD_PPI_EPPI1_CLIP 36
#define CMD_PPI_EPPI1_VDELAY 37
#define CMD_PPI_EPPI1_VCOUNT 38

#define CMD_PPI_FS1_EOL_BLANKING 39	/* FS1 End of Line Blanking */

#define CMD_PPI_GET_ALLCONFIG 40 /* For debug */

#define PPI_DMA_MAXSIZE	(64*1024)
#define PPI_READ_DELAY 1

#define CFG_PPI_XFR_TYPE_646_AF  0
#define CFG_PPI_XFR_TYPE_646_EF  1
#define CFG_PPI_XFR_TYPE_646_VB  2
#define CFG_PPI_XFR_TYPE_NON646  3

#define CFG_PPI_XFR_TYPE_NO_SYNC 0
#define CFG_PPI_XFR_TYPE_SYNC    3

/* Receive Modes */
#define CFG_PPI_PORT_CFG_XSYNC1  0
#define CFG_PPI_PORT_CFG_ISYNC23 1
#define CFG_PPI_PORT_CFG_XSYNC23 2
#define CFG_PPI_PORT_CFG_NOSYNC  3

/* Transmit Modes */
#define CFG_PPI_PORT_CFG_SYNC1	 0
#define CFG_PPI_PORT_CFG_SYNC23  1
#define CFG_PPI_PORT_CFG_NA      2
#define CFG_PPI_PORT_CFG_SYNC_FS2 3

#define CFG_PPI_FIELD_SELECT_1	   0
#define CFG_PPI_FIELD_SELECT_12	   1

/* Receive Mode */
#define CFG_PPI_FIELD_SELECT_XT    0
#define CFG_PPI_FIELD_SELECT_IT    1

#define CFG_PPI_PACK_DISABLE       0
#define CFG_PPI_PACK_ENABLE        1

#define CFG_PPI_SKIP_DISABLE       0
#define CFG_PPI_SKIP_ENABLE        1

#define CFG_PPI_SKIP_ODD           0
#define CFG_PPI_SKIP_EVEN          1

#if defined(PPI_CONTROL) || defined(PPI0_CONTROL)
#define CFG_PPI_DATALEN_8        0
#define CFG_PPI_DATALEN_10       1
#define CFG_PPI_DATALEN_11       2
#define CFG_PPI_DATALEN_12       3
#define CFG_PPI_DATALEN_13       4
#define CFG_PPI_DATALEN_14       5
#define CFG_PPI_DATALEN_15       6
#define CFG_PPI_DATALEN_16       7
#elif defined(EPPI0_CONTROL) || defined(EPPI1_CONTROL)
#define CFG_PPI_DATALEN_8        0
#define CFG_PPI_DATALEN_10       1
#define CFG_PPI_DATALEN_12       2
#define CFG_PPI_DATALEN_14       3
#define CFG_PPI_DATALEN_16       4
#define CFG_PPI_DATALEN_18       5
#define CFG_PPI_DATALEN_24       6
#endif

#define CFG_PPI_CLK_EDGE_RISE      0
#define CFG_PPI_CLK_EDGE_FALL      1

#define CFG_PPI_TRIG_EDGE_RISE      0
#define CFG_PPI_TRIG_EDGE_FALL      1

#define CFG_PPI_DIMS_UNDEF			0
#define	CFG_PPI_DIMS_1D				1
#define CFG_PPI_DIMS_2D				2

#endif /* __BFIN_PPI_H__ */
