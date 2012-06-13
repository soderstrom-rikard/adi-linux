/*
 * Copyright 2010-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _BFIN_ICC_H
#define _BFIN_ICC_H

#include <linux/cpumask.h>

struct icc_peer_platform_data {
	u32	peerid;
	u32	irq;
	u32	notify;
	u32 	phy_peer_mem;
};

struct icc_platform_data {
	u32	peer_count;
	struct icc_peer_platform_data *peer_info;
};

void platform_send_ipi(cpumask_t callmap, int irq);
void platform_send_ipi_cpu(unsigned int cpu, int irq);
void platform_clear_ipi(unsigned int cpu, int irq);
#endif
