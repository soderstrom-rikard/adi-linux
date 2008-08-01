/*
 * Common Blackfin startup code
 *
 * Copyright 2004-2008 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/blackfin.h>
#include <asm/trace.h>

/* A little BF561 glue ... */
#ifndef WDOG_CTL
# define WDOG_CTL WDOGA_CTL
#endif

__INIT

ENTRY(_real_start)
	/* Enable nested interrupts */
	[--sp] = reti;

	/* watchdog off for now */
	p0.l = lo(WDOG_CTL);
	p0.h = hi(WDOG_CTL);
	r0 = 0xAD6(z);
	w[p0] = r0;
	ssync;

	/* Code update for BSS size == 0
	 * Zero out the bss region.
	 */

	p1.l = ___bss_start;
	p1.h = ___bss_start;
	p2.l = ___bss_stop;
	p2.h = ___bss_stop;
	r0 = 0;
	p2 -= p1;
	lsetup (.L_clear_bss, .L_clear_bss) lc0 = p2;
.L_clear_bss:
	B[p1++] = r0;

	/* In case there is a NULL pointer reference
	 * Zero out region before stext
	 */

	p1.l = 0x0;
	p1.h = 0x0;
	r0.l = __stext;
	r0.h = __stext;
	r0 = r0 >> 1;
	p2 = r0;
	r0 = 0;
	lsetup (.L_clear_zero, .L_clear_zero) lc0 = p2;
.L_clear_zero:
	W[p1++] = r0;

	/* pass the uboot arguments to the global value command line */
	R0 = R7;
	call _cmdline_init;

	p1.l = __rambase;
	p1.h = __rambase;
	r0.l = __sdata;
	r0.h = __sdata;
	[p1] = r0;

	p1.l = __ramstart;
	p1.h = __ramstart;
	p3.l = ___bss_stop;
	p3.h = ___bss_stop;

	r1 = p3;
	[p1] = r1;

	/*
	 * load the current thread pointer and stack
	 */
	r1.l = _init_thread_union;
	r1.h = _init_thread_union;

	r2.l = 0x2000;
	r2.h = 0x0000;
	r1 = r1 + r2;
	sp = r1;
	usp = sp;
	fp = sp;
	jump.l _start_kernel;
ENDPROC(_real_start)

__FINIT
