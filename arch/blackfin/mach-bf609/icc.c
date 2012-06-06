#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <mach/pm.h>

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
