/*
 * Blackfin bf609 power management
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2
 */

#ifndef __MACH_BF609_PM_H__
#define __MACH_BF609_PM_H__

#include <linux/suspend.h>

struct bfin_cpu_pm_fns {
	void    (*save)(unsigned long *);
	void    (*restore)(unsigned long *);
	int     (*valid)(suspend_state_t state);
	void    (*enter)(suspend_state_t state);
	int     (*prepare)(void);
	void    (*finish)(void);
};

extern struct bfin_cpu_pm_fns *bfin_cpu_pm;

extern int bfin609_pm_enter(suspend_state_t state);
extern int bf609_pm_prepare(void);
extern void bf609_pm_finish(void);
#endif
