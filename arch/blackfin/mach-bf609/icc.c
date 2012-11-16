#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <mach/pm.h>
#include <asm/icc.h>

struct icc_peri_resource icc_peri_array[] = {
	{"cnt.0", NULL, 0},
	{"bfin-sdh.0", NULL, 0},
	{"bfin_can.0", NULL, 0},
	{"bfin-linkport.0", NULL, 0},
	{"bfin-linkport.1", NULL, 0},
	{"bfin-linkport.2", NULL, 0},
	{"bfin-linkport.3", NULL, 0},
	{"bfin-timer.0", NULL, 0},
	{"bfin-timer.1", NULL, 0},
	{"bfin-timer.2", NULL, 0},
	{"bfin-timer.3", NULL, 0},
	{"bfin-timer.4", NULL, 0},
	{"bfin-timer.5", NULL, 0},
	{"bfin-timer.6", NULL, 0},
	{"bfin-timer.7", NULL, 0},
	{"bfin-crc.0", NULL, 0},
	{"bfin-crc.1", NULL, 0},
	{"i2c-bfin-twi.0", NULL, 0},
	{"i2c-bfin-twi.1", NULL, 0},
	{"bfin-uart.0", NULL, 0},
	{"bfin-uart.1", NULL, 0},
	{"physmap-flash.0", NULL, 0},
	{"stmmaceth.0", NULL, 0},
	{"stmmaceth.1", NULL, 0},
	{"bfin-spi.0", NULL, 0},
	{"bfin-spi.1", NULL, 0},
	{"bfin-sport.0", NULL, 0},
	{"bfin-sport.1", NULL, 0},
	{"bfin-sport.2", NULL, 0},
	{"bfin-ppi.0", NULL, 0},
	{"bfin-ppi.1", NULL, 0},
	{"bfin-dma.0", NULL, 0},
	{"musb-blackfin.0", NULL, 0},
};

void platform_send_ipi_cpu(unsigned int cpu, int irq)
{
	BUG_ON(cpu >= 2);
	SSYNC();
	bfin_sec_raise_irq(irq);
	SSYNC();
}

void platform_send_ipi(cpumask_t callmap, int irq)
{
	unsigned int cpu;

	for_each_cpu_mask(cpu, callmap) {
		BUG_ON(cpu >= 2);
		SSYNC();
		platform_send_ipi_cpu(cpu, irq);
		SSYNC();
	}
}

void platform_clear_ipi(unsigned int cpu, int irq)
{
	BUG_ON(cpu >= 2);
}
