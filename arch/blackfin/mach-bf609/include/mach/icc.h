/*
 * Copyright 2010-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _MACH_BF609_ICC_H
#define _MACH_BF609_ICC_H
#include <mach/irq.h>

/* arch specific */
#define sm_atomic_read(v) bfin_read16(v)
#define sm_atomic_write(v, i) bfin_write16(v, i)

#define COREB_TASK_START	0x7C00000
#define COREB_MEMPOOL_START	0x7D00000
#define ICC_CODE_START		0xc8088000

#define COREB_ICC_LOW_SEND	IRQ_SOFT0
#define COREB_ICC_LOW_RECV	IRQ_SOFT1
#endif
