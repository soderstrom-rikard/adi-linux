/*
 * Blackfin Core B control driver
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <asm/pm.h>

#define CMD_COREB_START         _IO('b', 0)
#define CMD_COREB_STOP          _IO('b', 1)
#define CMD_COREB_RESET         _IO('b', 2)

static long coreb_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case CMD_COREB_START:
		bfin_coreb_start();
		break;
	case CMD_COREB_STOP:
		bfin_coreb_stop();
		break;
	case CMD_COREB_RESET:
		bfin_coreb_stop();
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct file_operations coreb_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = coreb_ioctl,
	.llseek         = noop_llseek,
};

static struct miscdevice coreb_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "coreb",
	.fops  = &coreb_fops,
};

static int __init bfin_coreb_init(void)
{
	return misc_register(&coreb_dev);
}

module_init(bfin_coreb_init);

MODULE_DESCRIPTION("Bfin Core B Support");
MODULE_LICENSE("GPL v2");
