/*
 * AD7160 touchscreen optional debug char type interface
 * for SELF and MUTUAL CDC results
 *
 * Copyright (C) 2010 Michael Hennerich, Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/kfifo.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>

#include <asm/uaccess.h>
#include <asm/system.h>

#include <linux/input/ad7160.h>
#include "ad7160.h"

/* Self CDC Result */
#define AFE_SLF_CDCVAL0		0x40051A00
#define MAX_NUM_SLF_CDC		28
/* Mutual CDC Results */
#define AFE_MTL_CDCVAL0		0x40051B80
#define MAX_NUM_MTL_CDC		182

#define AD7160_RAW_FIFO_DEPTH	10
#define AD7160_RAW_JUNK_SIZE	((MAX_NUM_SLF_CDC + MAX_NUM_MTL_CDC) * sizeof(unsigned int))

#define AD7160_RAW_FIFO_SIZE	(AD7160_RAW_FIFO_DEPTH * AD7160_RAW_JUNK_SIZE)

#define MAX_I2C_READNUM		63	/* Some I2C adaptors limitation */

/* NOTE:
 * It's assumed that only one instance of this driver,
 * with only one user is running the same time.
 * So it's ok to use global data structures
 */

static struct ad7160_raw_device {
	struct ad7160_bus_data	bdata;
	struct device *dev;
	struct mutex lock;
	struct kfifo fifo;
	spinlock_t fifo_lock;
	wait_queue_head_t waitq;
	struct fasync_struct *fifo_async;
	int open:1;	/* lock */
	unsigned int *buffer;
} ad7160_raw_device;

static int ad7160_read_multi(u32 reg, u32 len, u32 *data)
{
	unsigned readnum, count = 0;
	int ret;

	do {
		if (len > (count + MAX_I2C_READNUM))
			readnum = MAX_I2C_READNUM;
		else
			readnum = len - count;

		ret = ad7160_raw_device.bdata.bops->multi_read(
				ad7160_raw_device.bdata.client,
				reg + (count * sizeof(data)),
				readnum,
				&data[count]);
		count += readnum;

	} while (count < len);

	return ret;
}

void ad7160_feed_raw(void)
{
	int ret;

	if (!ad7160_raw_device.open)
		return;

	ret = ad7160_read_multi(AFE_SLF_CDCVAL0,
			MAX_NUM_SLF_CDC,
			&ad7160_raw_device.buffer[0]);

	ret = ad7160_read_multi(AFE_MTL_CDCVAL0,
			MAX_NUM_MTL_CDC,
			&ad7160_raw_device.buffer[MAX_NUM_SLF_CDC]);

	mutex_lock(&ad7160_raw_device.lock);
	ret = kfifo_in(&ad7160_raw_device.fifo,
			&ad7160_raw_device.buffer[0],
			AD7160_RAW_JUNK_SIZE);

	if (ret != AD7160_RAW_JUNK_SIZE) {
		dev_warn(ad7160_raw_device.dev,
			"FIFO Buffer Overflow: failed to put %lu bytes,"
			"fifo reached %d\n Resetting FIFO\n",
			AD7160_RAW_JUNK_SIZE - ret,
			kfifo_len(&ad7160_raw_device.fifo));

		kfifo_reset(&ad7160_raw_device.fifo);
		kfifo_in(&ad7160_raw_device.fifo,	/* try again */
			&ad7160_raw_device.buffer[0],
			AD7160_RAW_JUNK_SIZE);
	}

	kill_fasync(&ad7160_raw_device.fifo_async, SIGIO, POLL_IN);
	wake_up_interruptible(&ad7160_raw_device.waitq);
	mutex_unlock(&ad7160_raw_device.lock);

	return;
}

static int ad7160_raw_misc_fasync(int fd, struct file *filp, int on)
{
	return fasync_helper(fd, filp, on, &ad7160_raw_device.fifo_async);
}

static int ad7160_raw_misc_release(struct inode *inode, struct file *file)
{
	mutex_lock(&ad7160_raw_device.lock);
	ad7160_raw_device.open = false;
	mutex_unlock(&ad7160_raw_device.lock);

	return 0;
}

static int ad7160_raw_misc_open(struct inode *inode, struct file *file)
{
	mutex_lock(&ad7160_raw_device.lock);
	if (ad7160_raw_device.open) {
		mutex_unlock(&ad7160_raw_device.lock);
		return -EBUSY;
	}

	/* Flush input queue on open */
	kfifo_reset(&ad7160_raw_device.fifo);
	ad7160_raw_device.open = true;
	mutex_unlock(&ad7160_raw_device.lock);

	return 0;
}

static ssize_t ad7160_raw_misc_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	ssize_t ret;
	unsigned int copied;

	if ((kfifo_len(&ad7160_raw_device.fifo) == 0) &&
	    (file->f_flags & O_NONBLOCK))
		return -EAGAIN;

	ret = wait_event_interruptible(ad7160_raw_device.waitq,
				kfifo_len(&ad7160_raw_device.fifo) >=
				AD7160_RAW_JUNK_SIZE);
	if (ret)
		return ret;

	if (mutex_lock_interruptible(&ad7160_raw_device.lock))
		return -ERESTARTSYS;

	ret = kfifo_to_user(&ad7160_raw_device.fifo, buf, count, &copied);

	mutex_unlock(&ad7160_raw_device.lock);

	if (ret > 0) {
		struct inode *inode = file->f_path.dentry->d_inode;
		inode->i_atime = current_fs_time(inode->i_sb);
	}

	return ret ? ret : copied;
}

static unsigned int ad7160_raw_misc_poll(struct file *file, poll_table *wait)
{
	poll_wait(file, &ad7160_raw_device.waitq, wait);
	if (kfifo_len(&ad7160_raw_device.fifo))
		return POLLIN | POLLRDNORM;
	return 0;
}

static long ad7160_raw_misc_ioctl(struct file *fp,
			     unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	void __user *argp = (void __user *)arg;
	s32 val32;

	struct ad7160_iocreg_access reg_access;

	mutex_lock(&ad7160_raw_device.lock);
	switch (cmd) {
	case AD7160_RAW_IOCGREG:
		if (copy_from_user(&reg_access, argp, sizeof(reg_access))) {
			ret = -EFAULT;
			break;
		}

		val32 = ad7160_raw_device.bdata.bops->multi_read(
				ad7160_raw_device.bdata.client,
				reg_access.reg, 1, &reg_access.data);

		if (val32 < 0) {
			ret = -EIO;
			break;
		}

		if (copy_to_user(argp, &reg_access, sizeof(reg_access)))
			ret = -EFAULT;
		break;
	case AD7160_RAW_IOCSREG:
		if (copy_from_user(&reg_access, argp, sizeof(reg_access))) {
			ret = -EFAULT;
			break;
		}

		if (ad7160_raw_device.bdata.bops->write(
				ad7160_raw_device.bdata.client,
				reg_access.reg,
				reg_access.data) < 0) {
			ret = -EIO;
			break;
		}
		break;
	case AD7160_RAW_IOCGJUNKSIZE:
		val32 = AD7160_RAW_JUNK_SIZE;
		if (copy_to_user(argp, &val32, sizeof(val32)))
			ret = -EFAULT;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&ad7160_raw_device.lock);
	return ret;
}

static const struct file_operations ad7160_raw_misc_fops = {
	.owner		= THIS_MODULE,
	.read		= ad7160_raw_misc_read,
	.poll		= ad7160_raw_misc_poll,
	.open		= ad7160_raw_misc_open,
	.release	= ad7160_raw_misc_release,
	.fasync		= ad7160_raw_misc_fasync,
	.unlocked_ioctl	= ad7160_raw_misc_ioctl,
	.llseek		= no_llseek,
};

static struct miscdevice ad7160_raw_misc_device = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "ad7160_raw",
	.fops		= &ad7160_raw_misc_fops,
};

int
ad7160_probe_raw(struct device *dev, struct ad7160_bus_data *bdata,
		u32 devid, u16 bustype)
{
	int error;

	if (ad7160_raw_device.dev)
		return -EBUSY;

	spin_lock_init(&ad7160_raw_device.fifo_lock);
	error = kfifo_alloc(&ad7160_raw_device.fifo, AD7160_RAW_FIFO_SIZE, GFP_KERNEL);
	if (error) {
		dev_err(dev, "ad7160_raw: kfifo_alloc failed\n");
		return error;
	}

	ad7160_raw_device.buffer = kzalloc(AD7160_RAW_JUNK_SIZE, GFP_KERNEL);
	if (ad7160_raw_device.buffer == NULL) {
		dev_err(dev, "ad7160_raw: buffer alloc failed\n");
		goto err_kfifo_free;
	}

	init_waitqueue_head(&ad7160_raw_device.waitq);
	mutex_init(&ad7160_raw_device.lock);

	ad7160_raw_device.bdata = *bdata;
	ad7160_raw_device.dev = dev;

	error = misc_register(&ad7160_raw_misc_device);
	if (error) {
		dev_err(dev, "ad7160_raw: misc_register failed\n");
		goto err_free_buffer;
	}

	return 0;

 err_free_buffer:
	kfree(ad7160_raw_device.buffer);
 err_kfifo_free:
	kfifo_free(&ad7160_raw_device.fifo);
	ad7160_raw_device.dev = NULL;

	return error;
}

int ad7160_remove_raw(struct device *dev)
{
	if (ad7160_raw_device.dev) {
		misc_deregister(&ad7160_raw_misc_device);
		kfifo_free(&ad7160_raw_device.fifo);
		kfree(ad7160_raw_device.buffer);
		ad7160_raw_device.dev = NULL;
	}
	return 0;
}
