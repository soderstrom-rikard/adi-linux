/*
 * Copyright 2010-2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2
 */

#ifndef _BFIN_NMI_H_
#define _BFIN_NMI_H_

#include <linux/nmi.h>
#ifdef CONFIG_NMI_WATCHDOG
void check_nmi_watchdog(unsigned int cpu);
#else
static inline void check_nmi_watchdog(unsigned int cpu) {};
#endif

#endif
