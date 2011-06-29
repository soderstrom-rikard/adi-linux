/*
 * Analog Devices PPI header file
 *
 * Copyright (c) 2011 Scott Jiang <Scott.Jiang.Linux@gmail.com>
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

#ifndef _PPI_H_
#define _PPI_H_

#include <linux/interrupt.h>

/* control flags*/
#define PPI_FLAG_TX_MOD        0x0001 /* ppi in transmit mode */
#define PPI_FLAG_ACTIVE_FLD    0x0002 /* BT656 active field only */
#define PPI_FLAG_ENTIRE_FLD    0x0004 /* BT656 entire field */
#define PPI_FLAG_VBI           0x0008 /* BT656 VBI only */
#define PPI_FLAG_NO_SYNC       0x0010 /* no frame syncs */
#define PPI_FLAG_ONE_SYNC      0x0020 /* one frame sync */
#define PPI_FLAG_INTER_SYNC    0x0040 /* internal frame sync */
#define PPI_FLAG_FS2_ASSERT    0x0080 /* FS3 sync to FS2 assertion */
#define PPI_FLAG_FLD_SEL       0x0100 /* select both fields or internal trigger */
#define PPI_FLAG_PACK_EN       0x0200 /* packing mode enable */
#define PPI_FLAG_DMA32         0x0400 /* 32-bit DMA width enable */
#define PPI_FLAG_SKIP_EN       0x0800 /* skipping enabled */
#define PPI_FLAG_SKIP_EVEN     0x1000 /* skip even-numbered elements */
#define PPI_FLAG_CLK_FALL      0x2000 /* sample data on falling edge of PPI_CLK */
#define PPI_FLAG_FS_FALL       0x4000 /* FS1 and FS2 are falling edge asserted */
#define PPI_FLAG_BT656 \
	(PPI_FLAG_ACTIVE_FLD | PPI_FLAG_ENTIRE_FLD | PPI_FLAG_VBI)

struct ppi_if;

struct ppi_params {
	int width;
	int height;
	int bpp;
	unsigned long flags;
};

struct ppi_ops {
	int (*attach_irq)(struct ppi_if *intf, irq_handler_t handler);
	void (*detach_irq)(struct ppi_if *intf);
	int (*start)(struct ppi_if *intf);
	int (*stop)(struct ppi_if *intf);
	int (*set_params)(struct ppi_if *intf, struct ppi_params *params);
	void (*update_addr)(struct ppi_if *intf, unsigned long addr);
};

struct ppi_if {
	int dma_ch;
	int irq_err;
	int dma_config;
	int bytes_per_line;
	int lines_per_frame;
	unsigned short ppi_control;
	const unsigned short *pin_req;
	struct ppi_ops *ops;
	void *priv;
};

struct ppi_if *bfin_get_ppi_if(void);

#endif
