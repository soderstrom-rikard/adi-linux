#include <linux/init.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <asm/icc.h>

void icc_send_ipi_cpu(unsigned int cpu, int irq)
{
	platform_send_ipi_cpu(cpu, irq);
}

void icc_send_ipi_cpu_low(unsigned int cpu)
{
	platform_send_ipi_cpu(cpu, ICC_LOW_SEND);
}

void icc_send_ipi_cpu_high(unsigned int cpu)
{
	platform_send_ipi_cpu(cpu, ICC_HIGH_SEND);
}

void icc_clear_ipi_cpu(unsigned int cpu, int irq)
{
	platform_clear_ipi(cpu, irq);
}

irqreturn_t ipi_handler_int0(int irq, void *dev_instance)
{
	unsigned int cpu = blackfin_core_id();

	platform_clear_ipi(cpu, ICC_LOW_RECV);

	wakeup_icc_thread();
	return IRQ_HANDLED;
}

irqreturn_t ipi_handler_int1(int irq, void *dev_instance)
{
	unsigned int cpu = blackfin_core_id();

	platform_clear_ipi(cpu, ICC_HIGH_RECV);
	return IRQ_HANDLED;
}

static int __init icc_prepare_cpus(void)
{
	platform_request_ipi(ICC_LOW_RECV, ipi_handler_int0);
	platform_request_ipi(ICC_HIGH_RECV, ipi_handler_int1);
	return 0;
}

late_initcall(icc_prepare_cpus);
