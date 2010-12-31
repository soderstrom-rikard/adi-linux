#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

static const char supple0[] = "IRQ_SUPPLE_0";
static const char supple1[] = "IRQ_SUPPLE_1";

void wakeup_icc_thread(void);

void platform_request_ipi(int irq, void *handler)
{
	int ret;
	const char *name = (irq == IRQ_SUPPLE_0) ? supple0 : supple1;

	ret = request_irq(irq, handler, IRQF_DISABLED | IRQF_PERCPU, name, handler);
	if (ret)
		panic("Cannot request %s for IPI service", name);
}

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


irqreturn_t ipi_handler_int0(int irq, void *dev_instance)
{
	unsigned int cpu = blackfin_core_id();

	platform_clear_ipi(cpu, IRQ_SUPPLE_0);

	wakeup_icc_thread();
	return IRQ_HANDLED;
}

irqreturn_t ipi_handler_int1(int irq, void *dev_instance)
{
	unsigned int cpu = blackfin_core_id();

	platform_clear_ipi(cpu, IRQ_SUPPLE_1);
	return IRQ_HANDLED;
}

static int __init icc_prepare_cpus(void)
{
	platform_request_ipi(IRQ_SUPPLE_0, ipi_handler_int0);
	platform_request_ipi(IRQ_SUPPLE_1, ipi_handler_int1);
	return 0;
}

late_initcall(icc_prepare_cpus);
