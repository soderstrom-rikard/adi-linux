/*
 * arch/blackfin/kernel/reboot.c - handle shutdown/reboot
 *
 * Copyright 2004-2007 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/interrupt.h>
#include <asm/bfin-global.h>
#include <asm/reboot.h>

static inline void bfin_spin_forever(void)
{
	while (1)
		asm volatile ("idle");
}

/* A system soft reset makes external memory unusable
 * so force this function into L1.
 */
__attribute__((l1_text))
void bfin_reset(void)
{
	/* force BMODE and disable Core B (as needed) */
	bfin_write_SYSCR(0x20);
	while (1) {
		/* initiate system soft reset with magic 0x7 */
		bfin_write_SWRST(0x7);
		SSYNC();
		/* clear system soft reset */
		bfin_write_SWRST(0);
		SSYNC();
		/* issue core reset */
		asm("raise 1");
	}
}

__attribute__((weak))
void native_machine_restart(char *cmd)
{
}

void machine_restart(char *cmd)
{
	native_machine_restart(cmd);
	local_irq_disable();
	bfin_reset();
}

__attribute__((weak))
void native_machine_halt(void)
{
	local_irq_disable();
	bfin_spin_forever();
}

void machine_halt(void)
{
	native_machine_halt();
}

__attribute__((weak))
void native_machine_power_off(void)
{
	local_irq_disable();
	bfin_spin_forever();
}

void machine_power_off(void)
{
	native_machine_power_off();
}
