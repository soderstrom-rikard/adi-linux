/*
 * Copyright 2010-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _MACH_BF561_ICC_H
#define _MACH_BF561_ICC_H

/* arch specific */
#define sm_atomic_read(v) bfin_read16(v)
#define sm_atomic_write(v, i) bfin_write16(v, i)

typedef unsigned char sm_unit_t;
typedef unsigned short sm_uint16_t;
typedef unsigned long sm_uint32_t;
typedef sm_uint32_t sm_address_t;
typedef sm_uint16_t sm_atomic_t;
#define MSGQ_START_ADDR		0xFEB18000
#define MSGQ_SIZE		0x4000
#define DEBUG_MSG_BUF_ADDR	0xFEB1F000

#define COREB_TASK_START	0x3C00000
#define COREB_MEMPOOL_START	0x3D00000
#endif
