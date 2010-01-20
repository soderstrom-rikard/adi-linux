/*
 * Blackfin nmi_watchdog Driver
 *
 * Originally based on bfin_wdt.c
 * Copyright 2010-2010 Analog Devices Inc.
 * 		Graff Yang <graf.yang@analog.com>
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/hardirq.h>
#include <linux/sysdev.h>
#include <linux/pm.h>
#include <linux/nmi.h>
#include <asm/blackfin.h>
#include <asm/smp.h>
#include <asm/bitops.h>
#include <asm/atomic.h>
#include <asm/cacheflush.h>

/* Bit in WDOG_CTL that indicates watchdog has expired (WDR0) */
#define WDOG_EXPIRED 0x8000

/* Masks for WDEV field in WDOG_CTL register */
#define ICTL_RESET   0x0
#define ICTL_NMI     0x2
#define ICTL_GPI     0x4
#define ICTL_NONE    0x6
#define ICTL_MASK    0x6

/* Masks for WDEN field in WDOG_CTL register */
#define WDEN_MASK    0x0FF0
#define WDEN_ENABLE  0x0000
#define WDEN_DISABLE 0x0AD0

#define DRV_NAME "nmi-wdt"

#define NMI_WDT_TIMEOUT 5
static int nmi_wdt_cpu = 1;

static unsigned int timeout = NMI_WDT_TIMEOUT;
static int nmi_active;
static int panic_on_timeout;

static unsigned short wdoga_ctl;
static unsigned int wdoga_cnt;
static struct corelock_slot saved_corelock;
static atomic_t nmi_touched[NR_CPUS];

enum {
	COREA_ENTER_NMI = 0,
	COREA_EXIT_NMI,
	COREB_EXIT_NMI,

	NMI_EVENT_NR,
};
static unsigned long nmi_event __attribute__ ((__section__(".l2.bss")));

/* we are in nmi, non-atomic bit ops is safe */
static inline void set_nmi_event(int event)
{
	__set_bit(event, &nmi_event);
}

static inline void wait_nmi_event(int event)
{
	while (!test_bit(event, &nmi_event))
		barrier();
	__clear_bit(event, &nmi_event);
}

static inline void send_corea_nmi(void)
{
	wdoga_ctl = bfin_read_WDOGA_CTL();
	wdoga_cnt = bfin_read_WDOGA_CNT();

	bfin_write_WDOGA_CTL(WDEN_DISABLE);
	bfin_write_WDOGA_CNT(0);
	bfin_write_WDOGA_CTL(WDEN_ENABLE | ICTL_NMI);
}

static inline void restore_corea_nmi(void)
{
	bfin_write_WDOGA_CTL(WDEN_DISABLE);
	bfin_write_WDOGA_CTL(WDOG_EXPIRED | WDEN_DISABLE | ICTL_NONE);

	bfin_write_WDOGA_CNT(wdoga_cnt);
	bfin_write_WDOGA_CTL(wdoga_ctl);
}

static inline void save_corelock(void)
{
	saved_corelock = corelock;
	corelock.lock = 0;
}

static inline void restore_corelock(void)
{
	corelock = saved_corelock;
}


static inline void nmi_wdt_keepalive(void)
{
	bfin_write_WDOGB_STAT(0);
}

static inline void nmi_wdt_stop(void)
{
	bfin_write_WDOGB_CTL(WDEN_DISABLE);
}

/* before calling this function, you must stop the WDT */
static inline void nmi_wdt_clear(void)
{
	/* clear TRO bit, disable event generation */
	bfin_write_WDOGB_CTL(WDOG_EXPIRED | WDEN_DISABLE | ICTL_NONE);
}

static inline void  nmi_wdt_start(void)
{
	bfin_write_WDOGB_CTL(WDEN_ENABLE | ICTL_NMI);
}

static int nmi_wdt_running(void)
{
	return ((bfin_read_WDOGB_CTL() & WDEN_MASK) != WDEN_DISABLE);
}

static int nmi_wdt_set_timeout(unsigned long t)
{
	u32 cnt;
	int run;

	cnt = t * get_sclk();
	if (cnt < get_sclk()) {
		printk(KERN_WARNING "NMI: timeout value is too large\n");
		return -EINVAL;
	}

	run = nmi_wdt_running();
	nmi_wdt_stop();
	bfin_write_WDOGB_CNT(cnt);
	if (run)
		nmi_wdt_start();
	timeout = t;
	return 0;
}

static int __init init_nmi_wdt(void)
{
	nmi_wdt_set_timeout(timeout);
	nmi_wdt_start();
	nmi_active = true;
	printk(KERN_INFO "nmi_wdt: initialized: timeout=%d sec\n", timeout);
	return 0;
}
device_initcall(init_nmi_wdt);

static int __init setup_nmi_watchdog(char *str)
{
	if (!strncmp(str, "panic", 5))
		panic_on_timeout = true;

	return 1;
}
__setup("nmi_watchdog=", setup_nmi_watchdog);

/* Suspend/resume support */
#ifdef CONFIG_PM
static int nmi_wdt_suspend(struct sys_device *dev, pm_message_t state)
{
	nmi_wdt_stop();
	return 0;
}

static int nmi_wdt_resume(struct sys_device *dev)
{
	if (nmi_active)
		nmi_wdt_start();
	return 0;
}

static struct sysdev_class nmi_sysclass = {
	.name		= DRV_NAME,
	.resume		= nmi_wdt_resume,
	.suspend	= nmi_wdt_suspend,
};

static struct sys_device device_nmi_wdt = {
	.id	= 0,
	.cls	= &nmi_sysclass,
};

static int __init init_nmi_wdt_sysfs(void)
{
	int error;

	if (!nmi_active)
		return 0;

	error = sysdev_class_register(&nmi_sysclass);
	if (!error)
		error = sysdev_register(&device_nmi_wdt);
	return error;
}
late_initcall(init_nmi_wdt_sysfs);

#endif	/* CONFIG_PM */


void touch_nmi_watchdog(void)
{
	nmi_wdt_keepalive();
}

void check_nmi_watchdog(unsigned int cpu)
{
	atomic_set(&nmi_touched[cpu], 1);

	blackfin_dcache_invalidate_range(
		(unsigned long)(&nmi_touched[1 - cpu]),
			(unsigned long)(&nmi_touched[1 - cpu]));

	if (atomic_read(&nmi_touched[0]) == 1 &&
			atomic_read(&nmi_touched[1]) == 1) {
		atomic_set(&nmi_touched[0], 0);
		atomic_set(&nmi_touched[1], 0);
		touch_nmi_watchdog();
	}
}

asmlinkage notrace void do_nmi(struct pt_regs *fp)
{
	unsigned int cpu = smp_processor_id();
	nmi_enter();

	cpu_pda[cpu].__nmi_count += 1;

	if (cpu == nmi_wdt_cpu) {
		/* CoreB goes here first */

		/* reload the WDOG_STAT */
		nmi_wdt_keepalive();

		/* clear nmi interrupt for CoreB */
		nmi_wdt_stop();
		nmi_wdt_clear();

		/* trigger NMI interrupt of CoreA */
		send_corea_nmi();

		/* waiting CoreB to enter NMI */
		wait_nmi_event(COREA_ENTER_NMI);

		/* recover WDOGA's settings */
		restore_corea_nmi();

		save_corelock();

		/* corelock is save/cleared, CoreA is dummping messages */

		wait_nmi_event(COREA_EXIT_NMI);
	} else {
		/* OK, CoreA entered NMI */
		set_nmi_event(COREA_ENTER_NMI);
	}

	printk(KERN_EMERG "NMI Watchdog detected LOCKUP, dump for CPU %d\n",
							smp_processor_id());
	dump_bfin_process(fp);
	dump_bfin_mem(fp);
	show_regs(fp);
	dump_bfin_trace_buffer();
	show_stack(current, (unsigned long *)fp);
	printk(KERN_EMERG "\n\n");

	if (cpu == nmi_wdt_cpu) {
		if (!panic_on_timeout)
			nmi_wdt_start();

		/* CoreA dump finished, restore the corelock */
		restore_corelock();

		set_nmi_event(COREB_EXIT_NMI);
	} else {
		/* CoreB dump finished, notice the CoreA we are done */
		set_nmi_event(COREA_EXIT_NMI);

		/* synchronize with CoreA */
		wait_nmi_event(COREB_EXIT_NMI);
	}

	while (panic_on_timeout)
			barrier();
	nmi_exit();
}
