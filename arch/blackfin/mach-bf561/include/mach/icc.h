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

#define MSGQ_START_ADDR		0xFEB18000
#define MSGQ_SIZE		0x4000
#define DEBUG_MSG_BUF_ADDR	0xFEB1F000

#define COREB_TASK_START	0x3C00000
#define COREB_MEMPOOL_START	0x3D00000
#define ICC_CODE_START		0xFEB0000

#define ICC_LOW_SEND            IRQ_SUPPLE_0
#define ICC_LOW_RECV            IRQ_SUPPLE_0
#define ICC_HIGH_SEND           IRQ_SUPPLE_1
#define ICC_HIGH_RECV           IRQ_SUPPLE_1

#define COREB_ICC_LOW_SEND	IRQ_SUPPLE_0
#define COREB_ICC_LOW_RECV	IRQ_SUPPLE_0
#define COREB_ICC_HIGH_SEND	IRQ_SUPPLE_1
#define COREB_ICC_HIGH_RECV	IRQ_SUPPLE_1
#endif
