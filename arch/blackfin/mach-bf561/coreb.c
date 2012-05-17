/* Load firmware into Core B on a BF561
 *
 * Copyright 2004-2009 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

/* The Core B reset func requires code in the application that is loaded into
 * Core B.  In order to reset, the application needs to install an interrupt
 * handler for Supplemental Interrupt 0, that sets RETI to 0xff600000 and
 * writes bit 11 of SICB_SYSCR when bit 5 of SICA_SYSCR is 0.  This causes Core
 * B to stall when Supplemental Interrupt 0 is set, and will reset PC to
 * 0xff600000 when COREB_SRAM_INIT is cleared.
 */

#include <linux/module.h>

void bfin_coreb_start(void)
{
	bfin_write_SYSCR(bfin_read_SYSCR() & ~0x0020);
}

void bfin_coreb_stop(void)
{
	bfin_write_SYSCR(bfin_read_SYSCR() | 0x0020);
	bfin_write_SICB_SYSCR(bfin_read_SICB_SYSCR() | 0x0080);
}

void bfin_coreb_reset(void)
{
	bfin_write_SICB_SYSCR(bfin_read_SICB_SYSCR() | 0x0080);
}

MODULE_AUTHOR("Bas Vermeulen <bvermeul@blackstar.xs4all.nl>");
MODULE_DESCRIPTION("BF561 Core B Support");
MODULE_LICENSE("GPL");
