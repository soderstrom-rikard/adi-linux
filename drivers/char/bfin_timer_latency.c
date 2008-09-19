/*
 * File:         drivers/char/bfin_timer_latency.c
 * Based on:
 * Author:       Luke Yang
 *
 * Created:
 * Description:  Simple driver for testing interrupt latencies.
 *
 * Modified:
 *               Copyright 2005-2006 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/blackfin.h>
#include <asm/irq.h>

#undef DEBUG

#ifdef DEBUG
# define DPRINTK(x...)	printk(KERN_DEBUG x)
#else
# define DPRINTK(x...)	do { } while (0)
#endif

struct timer_latency_data_t {
	char value;
	unsigned long last_latency;
	unsigned long worst_latency;
	unsigned long average_latency;
	unsigned long test_number;
	unsigned int period_sclk;
	unsigned int period_cclk;
};

struct proc_dir_entry *timer_latency_file;
struct timer_latency_data_t timer_latency_data;

static int read_timer_latency(char *page, char **start,
			      off_t offset, int count, int *eof, void *data)
{
	return sprintf(page,
		       "number, worst latency, average latency\n  %lu,      %lu,       %lu\n",
		       timer_latency_data.test_number,
		       timer_latency_data.worst_latency,
		       timer_latency_data.average_latency);
}

static int write_timer_latency(struct file *file, const char *buffer,
			       unsigned long count, void *data)
{
	unsigned long sclk, cclk;
	char user_value[8];
	unsigned int wd_period_us;
	unsigned int period_sclk;
	unsigned int period_cclk;

	copy_from_user(user_value, buffer, count);
	wd_period_us = simple_strtoul(user_value, NULL, 0);

	if ((wd_period_us >= 100) && (timer_latency_data.value == 0)) {
		DPRINTK("start timer_latency\n");
		timer_latency_data.value = 1;
		sclk = get_sclk() / 1000000;
		cclk = get_cclk() / 1000000;

		/* convert from us to cycles */
		timer_latency_data.period_sclk = wd_period_us * sclk;
		timer_latency_data.period_cclk = wd_period_us * cclk;

		/* set count timer cycles */
		bfin_write_WDOG_CNT(timer_latency_data.period_sclk);

		/* set CYCLES counter to 0 and start it */
		__asm__(
			"R2 = 0;\n\t"
			"CYCLES = R2;\n\t"
			"CYCLES2 = R2;\n\t"
			"R2 = SYSCFG;\n\t"
			"BITSET(R2,1);\n\t"

			"P2.H = 0xffc0;\n\t"
			"P2.L = 0x0200;\n\t"
			"R3 = 0x0004;\n\t"
			"W[P2] = R3;\n\t"     /* start watchdog timer */
			"SYSCFG = R2;\n\t"    /* start cycles counter */
			: : : "R2", "R3", "P2"
		);

	}

	return 1;		/* always write 1 byte */
}

static irqreturn_t timer_latency_irq(int irq, void *dev_id)
{
	struct timer_latency_data_t *data = dev_id;

	unsigned long cycles_past, cclk;
	unsigned long latency;

	/* unsigned long first_latency, second_latency, third_latency; */

	/* get current cycle counter */
	/*
	   asm("%0 = CYCLES; p2 = 0xFFE07040; %1 = [p2]; p2 = 0xFFE07044; %2 = [p2]; p2 = 0xFFE07048; %3 = [p2];"
	   : "=d" (cycles_past), "=d" (first_latency), "=d" (second_latency), "=d" (third_latency):); */

      asm("%0 = CYCLES;":"=d"(cycles_past));

	bfin_write_WDOG_CTL(0x8AD6);	/* close counter */
	bfin_write_WDOG_CTL(0x8AD6);	/* have to write it twice to disable the timer */

	__asm__(                      /* stop CYCLES counter */
		"R2 = SYSCFG;\n\t"
		"BITCLR(R2,1);\n\t"
		"SYSCFG = R2;\n\t"
		: : : "R2"
	);

	latency = cycles_past - data->period_cclk;	/* latency in cycles */

	DPRINTK("latecy is %lu\n", latency);

	if (bfin_read_WDOG_STAT() != 0) {
		DPRINTK("timer_latency error!\n");
		return IRQ_HANDLED;
	}

	if (latency > data->worst_latency)
		data->worst_latency = latency;
	data->last_latency = latency;
	data->test_number++;
	data->average_latency =
	    ((data->average_latency * (data->test_number - 1)) +
	     latency) / (data->test_number);

	/* restart watchdog timer again */
	bfin_write_WDOG_CNT(data->period_sclk);

	__asm__(
		"R2 = 0;\n\t"
		"CYCLES = R2;\n\t"
		"CYCLES2 = R2;\n\t"
		"R2 = SYSCFG;\n\t"
		"BITSET(R2,1);\n\t"
		"P2.H = 0xffc0;\n\t"
		"P2.L = 0x0200;\n\t"
		"R3 = 0x0004;\n\t"
		"W[P2] = R3;\n\t"     /* start watchdog timer */
		"SYSCFG = R2;\n\t"    /* start cycles counter */
		: : : "R2", "R3", "P2"
		);

	return IRQ_HANDLED;
}

static int __init timer_latency_init(void)
{
	DPRINTK("timer_latency start!\n");

	timer_latency_file = create_proc_entry("timer_latency", 0666, NULL);
	if (timer_latency_file == NULL)
		return -ENOMEM;

	/* default value is 0 (timer is stopped) */
	timer_latency_data.value = 0;
	timer_latency_data.worst_latency = 0;
	timer_latency_data.average_latency = 0;
	timer_latency_data.last_latency = 0;

	timer_latency_file->data = &timer_latency_data;
	timer_latency_file->read_proc = &read_timer_latency;
	timer_latency_file->write_proc = &write_timer_latency;
	timer_latency_file->owner = THIS_MODULE;

	request_irq(IRQ_WATCH, timer_latency_irq, IRQF_DISABLED,
		    "timer_latency", &timer_latency_data);

	printk(KERN_INFO "timer_latency module loaded\n");

	return 0;		/* everything's OK */
}

static void __exit timer_latency_exit(void)
{
	remove_proc_entry("timer_latency", NULL);
	free_irq(IRQ_WATCH, NULL);
	printk(KERN_INFO "timer_latency module removed\n");
}

module_init(timer_latency_init);
module_exit(timer_latency_exit);

MODULE_AUTHOR("Luke Yang");
MODULE_DESCRIPTION("Timer Latency testing module");
MODULE_LICENSE("GPL");
