#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/icc.h>

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

int platform_res_manage_request_irq(uint16_t subid, unsigned int cpu)
{
	return 0;
}

void platform_res_manage_free_irq(uint16_t subid)
{
}
