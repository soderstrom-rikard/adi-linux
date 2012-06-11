/*
 * Copyright 2010-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _BFIN_ICC_H
#define _BFIN_ICC_H

#include <linux/cpumask.h>

struct icc_slave_platform_data {
	u32	irq;
	u32	notify;
};

struct icc_platform_data {
	u32	slave_count;
	struct icc_slave_platform_data *slave_info;
};

void platform_send_ipi(cpumask_t callmap, int irq);
void platform_send_ipi_cpu(unsigned int cpu, int irq);
void platform_clear_ipi(unsigned int cpu, int irq);
#endif
