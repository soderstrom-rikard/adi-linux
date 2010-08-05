/*
 * Blackfin SMP kernel icache flush anomaly walk around code in L1 SRAM
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/linkage.h>
#include <asm/blackfin.h>
#include <asm/cache.h>
#include <asm/page.h>

/* 05000491 - IFLUSH needs to reside in L1 memory */
#if ANOMALY_05000491

.section .l1.text

/* Invalidate all instruction cache lines assocoiated with this memory area */
ENTRY(_blackfin_icache_flush_range_l1)
	P0 = R0;

	R2 = -L1_CACHE_BYTES;

	/* start = (start & -L1_CACHE_BYTES) */
	R0 = R0 & R2;

	/* end = ((end - 1) & -L1_CACHE_BYTES) + L1_CACHE_BYTES; */
	R1 += -1;
	R1 = R1 & R2;
	R1 += L1_CACHE_BYTES;

	/* count = (end - start) >> L1_CACHE_SHIFT */
	R2 = R1 - R0;
	R2 >>= L1_CACHE_SHIFT;
	P1 = R2;

	P0 = R0;

	LSETUP (1f, 2f) LC1 = P1;
1:
/* 05000443 - IFLUSH cannot be last instruction in hardware loop */
#if ANOMALY_05000443
	IFLUSH [P0++];
2:	nop;
#else
2:	IFLUSH [P0++];
#endif

	RTS;
ENDPROC(_blackfin_icache_flush_range_l1)

#endif
