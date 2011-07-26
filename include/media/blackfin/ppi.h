/*
 * Analog Devices PPI header file
 *
 * Copyright (c) 2011 Analog Devices Inc.
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

struct ppi_if;

struct ppi_params {
	int width;
	int height;
	int bpp;
	unsigned short ppi_control;
};

struct ppi_ops {
	int (*attach_irq)(struct ppi_if *ppi, irq_handler_t handler);
	void (*detach_irq)(struct ppi_if *ppi);
	int (*start)(struct ppi_if *ppi);
	int (*stop)(struct ppi_if *ppi);
	int (*set_params)(struct ppi_if *ppi, struct ppi_params *params);
	void (*update_addr)(struct ppi_if *ppi, unsigned long addr);
};

struct ppi_info {
	const char *name; /* ppi or eppi */
	int dma_ch;
	int irq_err;
	unsigned long base;
	const unsigned short *pin_req;
};

struct ppi_if {
	int dma_config;
	int bytes_per_line;
	int lines_per_frame;
	unsigned short ppi_control;
	const struct ppi_ops *ops;
	const struct ppi_info *info;
	void *priv;
};

struct ppi_if *create_ppi_instance(const struct ppi_info *info);
void delete_ppi_instance(struct ppi_if *ppi);
#endif
