/*
 * Copyright 2007-2009 Analog Devices Inc.
 *               Graff Yang <graf.yang@analog.com>
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <asm/blackfin.h>
#include <asm/mem_map.h>
#include <asm/smp.h>

int hotplug_coreb;

void __cpuinit platform_cpu_die(void)
{
	unsigned long jump_add = &coreb_sleep;
	unsigned long iwr[2] = {0, 0};
	unsigned long bank = (IRQ_SUPPLE_0 - IVG_BASE) / 32;
	unsigned long bit = 1 << ((IRQ_SUPPLE_0 - IVG_BASE) % 32);

	hotplug_coreb = 1;

	iwr[bank] |= bit;

	/* disable core timer */
	bfin_write_TCNTL(0);

	/* clear ipi interrupt IRQ_SUPPLE_0 */
	bfin_write_SICB_SYSCR(bfin_read_SICB_SYSCR() | (1 << (10 + 1)));
	SSYNC();

	asm(
		"r0 = %0;\n"
		"r1 = %1;\n"
		"p0 = %2;\n"
		"jump (p0);"
		:
		: "r"(iwr[0]), "r"(iwr[1]), "p"(jump_add)
		: "R0", "R1", "P0"
	);
}
