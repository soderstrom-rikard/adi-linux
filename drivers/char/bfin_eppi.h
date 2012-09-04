/*
 * bfin_eppi.h generic blackfin eppi driver header file
 *
 * Copyright (c) 2012 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __BFIN_EPPI_H__
#define __BFIN_EPPI_H__

#define CMD_PPI_XFR_TYPE       1
#define CFG_PPI_XFR_TYPE_AV    0
#define CFG_PPI_XFR_TYPE_EF    1
#define CFG_PPI_XFR_TYPE_VB    2
#define CFG_PPI_XFR_TYPE_GP    3

#define CMD_PPI_FLD_SEL        2
#define CFG_PPI_FLD_MODE1      1

#define CMD_PPI_CLKGEN         3
#define CFG_PPI_INT_CLK        1

#define CMD_PPI_FSGEN          4
#define CFG_PPI_INT_FS         1

#define CMD_PPI_SIGNEXT        5
#define CFG_PPI_SIGN_EXT       1

#define CMD_PPI_POLC           6
#define CFG_PPI_DFALL_SFALL    0
#define CFG_PPI_DFALL_SRISE    1
#define CFG_PPI_DRISE_SFALL    2
#define CFG_PPI_DRISE_SRISE    3

#define CMD_PPI_POLS           7
#define CFG_PPI_FS1HI_FS2HI    0
#define CFG_PPI_FS1LO_FS2HI    1
#define CFG_PPI_FS1HI_FS2LO    2
#define CFG_PPI_FS1LO_FS2LO    3

#define CMD_PPI_DLEN           8
#define CFG_PPI_DATALEN_8      0
#define CFG_PPI_DATALEN_10     1
#define CFG_PPI_DATALEN_12     2
#define CFG_PPI_DATALEN_14     3
#define CFG_PPI_DATALEN_16     4
#define CFG_PPI_DATALEN_18     5
#define CFG_PPI_DATALEN_20     6
#define CFG_PPI_DATALEN_24     7

#define CMD_PPI_DMIRR          9
#define CFG_PPI_MIRR           1

#define CMD_PPI_SKIPEN         10
#define CFG_PPI_SKIP           1

#define CMD_PPI_SKIPEO         11
#define CFG_PPI_SKIP_EVEN      1

#define CMD_PPI_PACKEN         12
#define CFG_PPI_PACK           1

#define CMD_PPI_SWAPEN         13
#define CFG_PPI_SWAP           1

#define CMD_PPI_SPLTEO         14
#define CFG_PPI_SPLIT_EO       1

#define CMD_PPI_SUBSPLTODD     15
#define CFG_PPI_SUBSPLIT_ODD   1

#define CMD_PPI_SET_DIMS       16
#define CFG_PPI_DIMS_UNDEF     0
#define	CFG_PPI_DIMS_1D        1
#define CFG_PPI_DIMS_2D        2

#define CMD_PPI_LINE           17
#define CMD_PPI_FRAME          18
#define CMD_PPI_HDELAY         19
#define CMD_PPI_HCOUNT         20
#define CMD_PPI_VDELAY         21
#define CMD_PPI_VCOUNT         22
#define CMD_PPI_CLKDIV         23

#endif /* __BFIN_EPPI_H__ */
