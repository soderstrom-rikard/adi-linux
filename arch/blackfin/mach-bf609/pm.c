/*
 * Blackfin bf609 power management
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2
 */

#include <linux/suspend.h>
#include <linux/io.h>

#include <asm/dpmc.h>
#include <mach/pm.h>

void bfin_cpu_suspend(void)
{
	__asm__ __volatile__( \
			".align 8;" \
			"idle;" \
			: : \
			);
}

void bfin_deepsleep(unsigned long mask)
{
	uint32_t dpm0_ctl;

	dpm0_ctl = bfin_read32(DPM0_CTL);
	dpm0_ctl |= 0x00000004;
	bfin_write32(DPM0_CTL, dpm0_ctl);
	bfin_cpu_suspend();

}

void bfin_hibernate(unsigned long mask)
{
	uint32_t dpm0_ctl;

	dpm0_ctl = bfin_read32(DPM0_CTL);
	dpm0_ctl |= 0x00000010;
	bfin_write32(DPM0_CTL, dpm0_ctl);
	bfin_cpu_suspend();
}

void bf609_ddr_sr(void)
{
	uint32_t reg;

	reg = bfin_read_DDR0_CTL();
	reg |= 0x8;
	bfin_write_DDR0_CTL(reg);

	while (!(bfin_read_DDR0_STAT() & 0x8))
		continue;
}

void bf609_ddr_sr_exit(void)
{
	uint32_t reg;
	uint32_t dll_ctl;
	uint32_t dlldatacycle;

	reg = bfin_read_DDR0_CTL();
	reg |= 0x8;
	bfin_write_DDR0_CTL(reg);
	while (!(bfin_read_DDR0_STAT() & 0x4))
		continue;
	while (!(bfin_read_DDR0_STAT() & 0x8))
		continue;

	reg = bfin_read_DDR0_CTL();
	reg &= ~0x8;
	bfin_write_DDR0_CTL(reg);

	while ((bfin_read_DDR0_STAT() & 0x8))
		continue;

	dlldatacycle = (bfin_read_DDR0_STAT() & 0x00f00000) >> 20;
	dll_ctl = bfin_read_DDR0_DLLCTL();
	dll_ctl &= 0x0ff;
	bfin_write_DDR0_DLLCTL(dll_ctl | (dlldatacycle << 8));

	while (!(bfin_read_DDR0_STAT() & 0x2000))
		continue;

}

void bf609_cpu_pm_enter(suspend_state_t state)
{
	bfin_deepsleep(0xffff);
}

int bf609_cpu_pm_prepare(void)
{
	return 0;
}

void bf609_cpu_pm_finish(void)
{

}

static struct bfin_cpu_pm_fns bf609_cpu_pm = {
	.enter          = bf609_cpu_pm_enter,
	.prepare        = bf609_cpu_pm_prepare,
	.finish         = bf609_cpu_pm_finish,
};

static int __init bf609_init_pm(void)
{
	bfin_cpu_pm = &bf609_cpu_pm;
	return 0;
}

late_initcall(bf609_init_pm);
