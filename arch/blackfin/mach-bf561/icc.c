#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/icc.h>

struct icc_peri_resource icc_peri_array[] = {
	{"bfin-spi.0", NULL, 0},
	{"bfin-ppi.0", NULL, 0},
	{"bfin-ppi.1", NULL, 0},
	{"bfin-sport.0", NULL, 0},
	{"bfin-sport.1", NULL, 0},
	{"bfin-uart.0", NULL, 0},
	{"bfin-timer.0", NULL, 0},
};

void platform_send_ipi(cpumask_t callmap, int irq)
{
	unsigned int cpu;
	int offset = (irq == IRQ_SUPPLE_0) ? 6 : 8;

	for_each_cpu_mask(cpu, callmap) {
		BUG_ON(cpu >= 2);
		SSYNC();
		bfin_write_SICB_SYSCR(bfin_read_SICB_SYSCR() | (1 << (offset + cpu)));
		SSYNC();
	}
}

void platform_send_ipi_cpu(unsigned int cpu, int irq)
{
	int offset = (irq == IRQ_SUPPLE_0) ? 6 : 8;
	BUG_ON(cpu >= 2);
	SSYNC();
	bfin_write_SICB_SYSCR(bfin_read_SICB_SYSCR() | (1 << (offset + cpu)));
	SSYNC();
}

void platform_clear_ipi(unsigned int cpu, int irq)
{
	int offset = (irq == IRQ_SUPPLE_0) ? 10 : 12;
	BUG_ON(cpu >= 2);
	SSYNC();
	bfin_write_SICB_SYSCR(bfin_read_SICB_SYSCR() | (1 << (offset + cpu)));
	SSYNC();
}
