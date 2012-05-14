#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/icc.h>
#include <asm/pm.h>

char icc_low_name[] = "icc low irq";
char icc_high_name[] = "icc high irq";

void platform_request_ipi(int irq, void *handler)
{
	int ret;
	const char *name = (irq == ICC_LOW_RECV) ? icc_low_name : icc_high_name;

	ret = request_irq(irq, handler, IRQF_DISABLED | IRQF_PERCPU,
				name, handler);
	if (ret)
		panic("Cannot request %s for IPI service", name);
}

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
