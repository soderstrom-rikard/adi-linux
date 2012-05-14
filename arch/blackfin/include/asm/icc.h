/*
 * Copyright 2010-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _BFIN_ICC_H
#define _BFIN_ICC_H

#include <mach/icc.h>

void platform_request_ipi(int irq, void *handler);
void platform_send_ipi(cpumask_t callmap, int irq);
void platform_send_ipi_cpu(unsigned int cpu, int irq);
void platform_clear_ipi(unsigned int cpu, int irq);

void wakeup_icc_thread(void);
#endif
