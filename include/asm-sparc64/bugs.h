/*  $Id$
 *  include/asm-sparc64/bugs.h:  Sparc probes for various bugs.
 *
 *  Copyright (C) 1996 David S. Miller (davem@caip.rutgers.edu)
 */


extern unsigned long loops_per_jiffy;

static void __init check_bugs(void)
{
#ifndef CONFIG_SMP
	cpu_data(0).udelay_val = loops_per_jiffy;
#endif
}
