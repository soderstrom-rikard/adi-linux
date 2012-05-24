/*
 * Copyright 2010-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _MACH_BF609_ICC_H
#define _MACH_BF609_ICC_H

/* arch specific */
#define sm_atomic_read(v) bfin_read16(v)
#define sm_atomic_write(v, i) bfin_write16(v, i)

#define MSGQ_START_ADDR		(L2_START + 0x10000)
#define MSGQ_SIZE		0x4000
#define DEBUG_MSG_BUF_ADDR	(MSGQ_START_ADDR + MSGQ_SIZE + 0x1000)

#define COREB_TASK_START	0x7C00000
#define COREB_MEMPOOL_START	0x7D00000

#define ICC_LOW_SEND		IRQ_SOFT1
#define ICC_LOW_RECV		IRQ_SOFT0
#define ICC_HIGH_SEND		IRQ_SOFT3
#define ICC_HIGH_RECV		IRQ_SOFT2
#endif
