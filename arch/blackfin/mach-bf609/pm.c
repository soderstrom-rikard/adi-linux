/*
 * Blackfin bf609 power management
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2
 */

#include <linux/suspend.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

#include <linux/delay.h>

#include <asm/dpmc.h>
#include <asm/pm.h>
#include <mach/pm.h>

void bfin_cpu_suspend(void)
{
	__asm__ __volatile__( \
			".align 8;" \
			"idle;" \
			: : \
			);
}

void pm_dump_reg(void)
{
	uint32_t reg;
	reg = bfin_read32(SEC_SCTL1);
	printk("SEC_SCTL1 %08x %08x\n", SEC_SCTL1, reg);
}

void bfin_deepsleep(unsigned long mask)
{
	uint32_t dpm0_ctl;

	bfin_write32(DPM0_WAKE_EN, mask);

	dpm0_ctl = bfin_read32(DPM0_CTL);
	dpm0_ctl = 0x00000008;
	bfin_write32(DPM0_CTL, dpm0_ctl);
	bfin_write32(DPM0_WAKE_EN, mask);
	bfin_write32(DPM0_WAKE_POL, 0);
	SSYNC();
	pm_dump_reg();
#if 1
	while (1) {
		mdelay(500);
		printk("idle\n");
	}
#endif
	bfin_cpu_suspend();

}

void bfin_hibernate(unsigned long mask)
{
	uint32_t dpm0_ctl;
	int i = 100000000;

	bfin_write32(DPM0_WAKE_EN, mask);
	bfin_write32(DPM0_WAKE_POL, 0);
	bfin_write32(DPM0_RESTORE0, 0xFFAABBCC);
	bfin_write32(DPM0_PGCNTR, 0x0000FFFF);
	bfin_write32(DPM0_HIB_DIS, 0xFFFF);
	SSYNC();

	printk("hibernate: restore %x pgcnt %x\n", bfin_read32(DPM0_RESTORE0), bfin_read32(DPM0_PGCNTR));

	dpm0_ctl = bfin_read32(DPM0_CTL);
	dpm0_ctl = 0x00000010;
	bfin_write32(DPM0_CTL, dpm0_ctl);
	SSYNC();

#if 1
	while (1) {
		mdelay(500);
		printk("idle\n");
	}
#endif

	bf609_hibernate();
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
	int error;
	error = irq_set_irq_wake(255, 1);
	if (error < 0)
		printk("Unable to get irq wake\n");

	if (state == PM_SUSPEND_STANDBY)
		bfin_deepsleep(0xffff);
	else
		bfin_hibernate(0xffff);
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

static irqreturn_t test_isr(int irq, void *dev_id)
{
	printk("gpio irq %d\n", irq);
	return IRQ_HANDLED;
}

static irqreturn_t dummy_isr(int irq, void *dev_id)
{
	printk("enter %s\n", __func__);
	return IRQ_HANDLED;
}

static int __init bf609_init_pm(void)
{
	int irq;
	int error;
	error = gpio_request(GPIO_PG4, "gpiopg4");
	if (error < 0) {
		printk("failed to request GPIO %d, error %d\n",
				GPIO_PG4, error);
	}

	error = gpio_direction_input(GPIO_PG4);
	if (error < 0) {
		printk("failed to configure"
				" direction for GPIO %d, error %d\n",
				GPIO_PG4, error);
	}

	irq = gpio_to_irq(GPIO_PG4);
	if (irq < 0) {
		error = irq;
		printk("Unable to get irq number for GPIO %d, error %d\n",
				GPIO_PG4, error);
	}

	printk("%s gpio %d irq %d\n", __func__, GPIO_PG4, irq);

	error = request_irq(irq, test_isr, IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND, "gpiopg4", NULL);
	if (error < 0)
		printk("Unable to get irq\n");

	error = request_irq(IRQ_CGU_EVT, dummy_isr, IRQF_NO_SUSPEND, "cgu0 event", NULL);
	if (error < 0)
		printk("Unable to get irq\n");

	error = request_irq(IRQ_DPM, dummy_isr, IRQF_NO_SUSPEND, "dpm0 event", NULL);
	if (error < 0)
		printk("Unable to get irq\n");


	bfin_cpu_pm = &bf609_cpu_pm;
	return 0;
}

late_initcall(bf609_init_pm);
