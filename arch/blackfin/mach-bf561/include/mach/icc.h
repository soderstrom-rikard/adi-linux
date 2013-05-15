/*
 * Copyright 2010-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _MACH_BF561_ICC_H
#define _MACH_BF561_ICC_H
#include <mach/irq.h>

/* arch specific */
#define sm_atomic_read(v) bfin_read16(v)
#define sm_atomic_write(v, i) bfin_write16(v, i)

#define COREB_TASK_START	0x3C00000	/* core B DRAM resion should be at the boundary of 4M bytes */
#define COREB_TASK_MEM_SIZE	0x400000	/* core B DRAM size shold be multiple of 4M bytes */
#define COREB_MEMPOOL_START	0x3D00000
#define ICC_CODE_START		0xFEB08000

#define COREB_ICC_LOW_SEND	IRQ_SUPPLE_0
#define COREB_ICC_LOW_RECV	IRQ_SUPPLE_0
#endif
