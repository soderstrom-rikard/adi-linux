/*
 * Simple timer driver
 *
 * Author: (C) 2006 by Axel Weiss (awe@aglaia-gmbh.de)
 *
 * This is a simple char-device interface driver for the bf5xx_timers driver.
 * It primarily serves as an example for how to use the hardware drivers
 * on blackfin, but may also be used as a starting point of development
 * for more sophisticated driver frontends.
 *
 * Behaviour
 * With this driver, a device node /dev/bf5xx_timer[0...] with major number
 * 238 and minor number 0... can be used to access one of blackfin's internal
 * hardware timer. After open(), the timer may be accessed via ioctl:
 *	BFIN_SIMPLE_TIMER_SET_PERIOD: set timer period (in microseconds)
 *	BFIN_SIMPLE_TIMER_START: start timer
 *	BFIN_SIMPLE_TIMER_STOP: stop timer
 *	BFIN_SIMPLE_TIMER_READ: read the numbers of periods (irq-count)
 * This driver enables:
 *	no physical timer output (OUT_DIS is set)
 *	free running from start
 *	timer interrupt, counting
 * The driver opens a (ro) file at /proc/bfin_simple_timer that shows the
 * irq count values for all timers.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <asm/gptimers.h>
#include <asm/irq.h>
#include <asm/bfin_simple_timer.h>
#include <asm/bfin-global.h>
#include <asm/portmux.h>

#define TIMER_MAJOR 238
#define DRV_NAME    "bfin_simple_timer"

MODULE_AUTHOR("Axel Weiss <awe@aglaia-gmbh.de>");
MODULE_DESCRIPTION("simple timer char-device interface for Blackfin gptimers");
MODULE_LICENSE("GPL");

struct timer {
	unsigned short id, bit;
	unsigned short per_pin;
	unsigned long irqbit, isr_count;
	int irq;
};

static struct timer timer_code[MAX_BLACKFIN_GPTIMERS] = {
	{TIMER0_id,  TIMER0bit,  P_TMR0, TIMER_STATUS_TIMIL0, 0, IRQ_TIMER0},
	{TIMER1_id,  TIMER1bit,  P_TMR1, TIMER_STATUS_TIMIL1, 0, IRQ_TIMER1},
	{TIMER2_id,  TIMER2bit,  P_TMR2, TIMER_STATUS_TIMIL2, 0, IRQ_TIMER2},
#if (MAX_BLACKFIN_GPTIMERS > 3)
	{TIMER3_id,  TIMER3bit,  P_TMR3, TIMER_STATUS_TIMIL3, 0, IRQ_TIMER3},
	{TIMER4_id,  TIMER4bit,  P_TMR4, TIMER_STATUS_TIMIL4, 0, IRQ_TIMER4},
	{TIMER5_id,  TIMER5bit,  P_TMR5, TIMER_STATUS_TIMIL5, 0, IRQ_TIMER5},
	{TIMER6_id,  TIMER6bit,  P_TMR6, TIMER_STATUS_TIMIL6, 0, IRQ_TIMER6},
	{TIMER7_id,  TIMER7bit,  P_TMR7, TIMER_STATUS_TIMIL7, 0, IRQ_TIMER7},
#endif
#if (MAX_BLACKFIN_GPTIMERS > 8)
	{TIMER8_id,  TIMER8bit,  P_TMR8, TIMER_STATUS_TIMIL8, 0, IRQ_TIMER8},
	{TIMER9_id,  TIMER9bit,  P_TMR9, TIMER_STATUS_TIMIL9, 0, IRQ_TIMER9},
	{TIMER10_id, TIMER10bit, P_TMR10, TIMER_STATUS_TIMIL10, 0, IRQ_TIMER10},
#if (MAX_BLACKFIN_GPTIMERS > 11)
	{TIMER11_id, TIMER11bit, P_TMR11, TIMER_STATUS_TIMIL11, 0, IRQ_TIMER11},
#endif
#endif
};

static long
timer_ioctl(struct file *filp, uint cmd, unsigned long arg)
{
	struct timer *t = filp->private_data;
	unsigned long period = 0, width = 0, mode = 0;
	switch (cmd) {
	case BFIN_SIMPLE_TIMER_SET_PERIOD:
		if (arg < 2)
			return -EFAULT;
		period = arg;
		set_gptimer_period(t->id, period);
		pr_debug(DRV_NAME ": TIMER_SET_PERIOD: arg=%lu, period=%lu\n",
			arg, period);
		break;
	case BFIN_SIMPLE_TIMER_SET_WIDTH:
		width = arg;
		set_gptimer_pwidth(t->id, width);
		pr_debug(DRV_NAME ": TIMER_SET_WIDTH: arg=%lu, width=%lu\n",
			arg, width);
		break;
	case BFIN_SIMPLE_TIMER_SET_MODE:
		mode = arg;
		switch (mode) {
			pr_debug(DRV_NAME ": TIMER_SET_MODE: mode %lu\n", mode);
		case BFIN_SIMPLE_TIMER_MODE_PWM_ONESHOT:
#ifdef CONFIG_BF60x
			set_gptimer_config(t->id,  TIMER_OUT_DIS | TIMER_MODE_PWM
					| TIMER_PULSE_HI | TIMER_IRQ_WID_DLY);
#else
			set_gptimer_config(t->id, OUT_DIS | PWM_OUT | PERIOD_CNT | IRQ_ENA);
#endif
			break;
		case BFIN_SIMPLE_TIMER_MODE_PWMOUT_CONT:
#ifdef CONFIG_BF60x
			set_gptimer_config(t->id,  TIMER_MODE_PWM_CONT
						| TIMER_PULSE_HI | TIMER_IRQ_PER);
#else
			set_gptimer_config(t->id, PWM_OUT | PERIOD_CNT | IRQ_ENA);
#endif
			break;
		case BFIN_SIMPLE_TIMER_MODE_WDTH_CAP:
#ifdef CONFIG_BF60x
			set_gptimer_config(t->id, TIMER_MODE_WDTH | TIMER_IRQ_PER);
#else
			set_gptimer_config(t->id, WDTH_CAP | PERIOD_CNT | IRQ_ENA);
#endif
			break;
		case BFIN_SIMPLE_TIMER_MODE_PWMOUT_CONT_NOIRQ:
#ifdef CONFIG_BF60x
			set_gptimer_config(t->id,  TIMER_MODE_PWM_CONT
						| TIMER_PULSE_HI);
#else
			set_gptimer_config(t->id, PWM_OUT | PERIOD_CNT);
#endif
			break;
		default:
			pr_debug(DRV_NAME ": error mode\n");
		}
		break;
	case BFIN_SIMPLE_TIMER_START:
		period = get_gptimer_period(t->id);
		width = get_gptimer_pwidth(t->id);
		pr_debug(DRV_NAME ": TIMER_START: period=%lu, width=%lu\n",
				period, width);
		enable_gptimers(t->bit);
		break;
	case BFIN_SIMPLE_TIMER_STOP:
		disable_gptimers(t->bit);
		break;
	case BFIN_SIMPLE_TIMER_READ:
		put_user(t->isr_count, (unsigned long __user *)arg);
		break;
	case BFIN_SIMPLE_TIMER_READ_COUNTER:
		put_user(get_gptimer_count(t->id), (unsigned long __user *)arg);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static irqreturn_t
timer_isr(int irq, void *dev_id)
{
	int minor = (int)dev_id;
	struct timer *t = &timer_code[minor];
	if (get_gptimer_intr(t->id)) {
		clear_gptimer_intr(t->id);
		t->isr_count++;
	}
	return IRQ_HANDLED;
}

static int
timer_open(struct inode *inode, struct file *filp)
{
	int minor = MINOR(inode->i_rdev);
	struct timer *t;
	int err = 0;

	if (minor >= MAX_BLACKFIN_GPTIMERS)
		return -ENODEV;

	t = &timer_code[minor];
	filp->private_data = t;

	err = request_irq(t->irq, timer_isr, IRQF_DISABLED, DRV_NAME, (void *)minor);
	if (err < 0) {
		printk(KERN_ERR "request_irq(%d) failed\n", t->irq);
		return err;
	}

	err = pinmux_request(t->per_pin, "timer test");
	if (err) {
		printk(KERN_ERR "request pin(%d) failed\n", t->per_pin);
		free_irq(t->irq, (void *)minor);
		return err;
	}

	pr_debug(DRV_NAME ": device(%d) opened\n", minor);

	return 0;
}

static int
timer_close(struct inode *inode, struct file *filp)
{
	int minor = MINOR(inode->i_rdev);
	struct timer *t = filp->private_data;;
	disable_gptimers(t->bit);
	pinmux_free(t->per_pin);
	free_irq(t->irq, (void *)minor);
	pr_debug(DRV_NAME ": device(%d) closed\n", minor);
	return 0;
}

static int
timer_read_proc(struct seq_file *m, void *v)
{
	int i;

	seq_printf(m, "sclk = %lu\n", get_sclk());
	for (i = 0; i < MAX_BLACKFIN_GPTIMERS; ++i)
		seq_printf(m, "timer %2d isr count: %lu\n",
			i, timer_code[i].isr_count);
	return 0;
}

static int
timer_open_proc(struct inode *inode, struct file *file)
{
	return single_open(file, timer_read_proc, NULL);
}

static ssize_t
timer_status_show(struct class *timer_class, struct class_attribute *attr,
                  char *buf)
{
	char *p;
	unsigned short i;
	p = buf;

	for (i = 0; i < MAX_BLACKFIN_GPTIMERS; ++i)
		p += sprintf(p, "timer %2d isr count: %lu\n",
			i, timer_code[i].isr_count);

	return p - buf;
}

static const struct file_operations fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = timer_ioctl,
	.open           = timer_open,
	.release        = timer_close,
};

static const struct file_operations timer_proc_fops = {
	.open = timer_open_proc,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static struct proc_dir_entry *timer_dir_entry;
static struct class *timer_class;
static CLASS_ATTR(status, S_IRUGO, &timer_status_show, NULL);

static int __init timer_initialize(void)
{
	int minor;
	int err;

	err = register_chrdev(TIMER_MAJOR, DRV_NAME, &fops);
	if (err < 0) {
		pr_debug(DRV_NAME ": could not register device %s\n", DRV_NAME);
		return err;
	}

	timer_dir_entry = proc_create(DRV_NAME, 0444, NULL, &timer_proc_fops);
	if (!timer_dir_entry)
		return -ENOMEM;

	timer_class = class_create(THIS_MODULE, "timer");
	err = class_create_file(timer_class, &class_attr_status);
	if (err) {
		remove_proc_entry(DRV_NAME, NULL);
		unregister_chrdev(TIMER_MAJOR, DRV_NAME);
		return err;
	}

	minor = 0;
#ifdef CONFIG_TICK_SOURCE_SYSTMR0
	/* gptimer0 is used to generate the tick interrupt */
	++minor;
#endif
	for (minor = 0; minor < MAX_BLACKFIN_GPTIMERS; minor++)
		device_create(timer_class, NULL, MKDEV(TIMER_MAJOR, minor),
			NULL, "timer%d", minor);

	pr_debug(DRV_NAME ": module loaded\n");
	return 0;
}
module_init(timer_initialize);

static void __exit timer_cleanup(void)
{
	remove_proc_entry(DRV_NAME, NULL);
	unregister_chrdev(TIMER_MAJOR, DRV_NAME);
	pr_debug(DRV_NAME ": module unloaded\n");
}
module_exit(timer_cleanup);
