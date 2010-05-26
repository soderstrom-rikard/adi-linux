/*
 * AD7879/AD7889 based touchscreen and GPIO driver
 *
 * Copyright (C) 2008-2010 Michael Hennerich, Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 *
 * History:
 * Copyright (c) 2005 David Brownell
 * Copyright (c) 2006 Nokia Corporation
 * Various changes: Imre Deak <imre.deak@nokia.com>
 *
 * Using code from:
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *  - ad7877.c
 *	Copyright (C) 2006-2008 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <linux/spi/ad7879.h>
#include "ad7879.h"

#define AD7879_REG_ZEROS		0
#define AD7879_REG_CTRL1		1
#define AD7879_REG_CTRL2		2
#define AD7879_REG_CTRL3		3
#define AD7879_REG_AUX1HIGH		4
#define AD7879_REG_AUX1LOW		5
#define AD7879_REG_TEMP1HIGH		6
#define AD7879_REG_TEMP1LOW		7
#define AD7879_REG_XPLUS		8
#define AD7879_REG_YPLUS		9
#define AD7879_REG_Z1			10
#define AD7879_REG_Z2			11
#define AD7879_REG_AUXVBAT		12
#define AD7879_REG_TEMP			13
#define AD7879_REG_REVID		14

/* Control REG 1 */
#define AD7879_TMR(x)			((x & 0xFF) << 0)
#define AD7879_ACQ(x)			((x & 0x3) << 8)
#define AD7879_MODE_NOC			(0 << 10)	/* Do not convert */
#define AD7879_MODE_SCC			(1 << 10)	/* Single channel conversion */
#define AD7879_MODE_SEQ0		(2 << 10)	/* Sequence 0 in Slave Mode */
#define AD7879_MODE_SEQ1		(3 << 10)	/* Sequence 1 in Master Mode */
#define AD7879_MODE_INT			(1 << 15)	/* PENIRQ disabled INT enabled */

/* Control REG 2 */
#define AD7879_FCD(x)			((x & 0x3) << 0)
#define AD7879_RESET			(1 << 4)
#define AD7879_MFS(x)			((x & 0x3) << 5)
#define AD7879_AVG(x)			((x & 0x3) << 7)
#define	AD7879_SER			(1 << 9)	/* non-differential */
#define	AD7879_DFR			(0 << 9)	/* differential */
#define AD7879_GPIOPOL			(1 << 10)
#define AD7879_GPIODIR			(1 << 11)
#define AD7879_GPIO_DATA		(1 << 12)
#define AD7879_GPIO_EN			(1 << 13)
#define AD7879_PM(x)			((x & 0x3) << 14)
#define AD7879_PM_SHUTDOWN		(0)
#define AD7879_PM_DYN			(1)
#define AD7879_PM_FULLON		(2)

/* Control REG 3 */
#define AD7879_TEMPMASK_BIT		(1<<15)
#define AD7879_AUXVBATMASK_BIT		(1<<14)
#define AD7879_INTMODE_BIT		(1<<13)
#define AD7879_GPIOALERTMASK_BIT	(1<<12)
#define AD7879_AUXLOW_BIT		(1<<11)
#define AD7879_AUXHIGH_BIT		(1<<10)
#define AD7879_TEMPLOW_BIT		(1<<9)
#define AD7879_TEMPHIGH_BIT		(1<<8)
#define AD7879_YPLUS_BIT		(1<<7)
#define AD7879_XPLUS_BIT		(1<<6)
#define AD7879_Z1_BIT			(1<<5)
#define AD7879_Z2_BIT			(1<<4)
#define AD7879_AUX_BIT			(1<<3)
#define AD7879_VBAT_BIT			(1<<2)
#define AD7879_TEMP_BIT			(1<<1)

enum {
	AD7879_SEQ_XPOS  = 0,
	AD7879_SEQ_YPOS  = 1,
	AD7879_SEQ_Z1    = 2,
	AD7879_SEQ_Z2    = 3,
	AD7879_NR_SENSE  = 4,
};

#define	MAX_12BIT			((1<<12)-1)
#define	TS_PEN_UP_TIMEOUT		msecs_to_jiffies(50)

struct ad7879 {
	struct ad7879_bus_data	bdata;
	struct input_dev	*input;
	struct work_struct	work;
	struct timer_list	timer;
#ifdef CONFIG_GPIOLIB
	struct gpio_chip	gc;
#endif
	struct mutex		mutex;
	unsigned		disabled:1;	/* P: mutex */
	u16			conversion_data[AD7879_NR_SENSE];
	char			phys[32];
	u8			first_conversion_delay;
	u8			acquisition_time;
	u8			averaging;
	u8			pen_down_acc_interval;
	u8			median;
	u16			x_plate_ohms;
	u16			pressure_max;
	u16			cmd_crtl1;
	u16			cmd_crtl2;
	u16			cmd_crtl3;
};

static int ad7879_read(struct ad7879 *ts, u8 reg)
{
	return ts->bdata.bops->read(ts->bdata.client, reg);
}
static int ad7879_multi_read(struct ad7879 *ts, u8 first_reg, u8 count, u16 *buf)
{
	return ts->bdata.bops->multi_read(ts->bdata.client, first_reg, count, buf);
}
static int ad7879_write(struct ad7879 *ts, u8 reg, u16 val)
{
	return ts->bdata.bops->write(ts->bdata.client, reg, val);
}

static int ad7879_report(struct ad7879 *ts)
{
	struct input_dev *input_dev = ts->input;
	unsigned Rt;
	u16 x, y, z1, z2;

	x = ts->conversion_data[AD7879_SEQ_XPOS] & MAX_12BIT;
	y = ts->conversion_data[AD7879_SEQ_YPOS] & MAX_12BIT;
	z1 = ts->conversion_data[AD7879_SEQ_Z1] & MAX_12BIT;
	z2 = ts->conversion_data[AD7879_SEQ_Z2] & MAX_12BIT;

	/*
	 * The samples processed here are already preprocessed by the AD7879.
	 * The preprocessing function consists of a median and an averaging filter.
	 * The combination of these two techniques provides a robust solution,
	 * discarding the spurious noise in the signal and keeping only the data of interest.
	 * The size of both filters is programmable. (dev.platform_data, see linux/spi/ad7879.h)
	 * Other user-programmable conversion controls include variable acquisition time,
	 * and first conversion delay. Up to 16 averages can be taken per conversion.
	 */

	if (likely(x && z1)) {
		/* compute touch pressure resistance using equation #1 */
		Rt = (z2 - z1) * x * ts->x_plate_ohms;
		Rt /= z1;
		Rt = (Rt + 2047) >> 12;

		if (!timer_pending(&ts->timer))
			input_report_key(input_dev, BTN_TOUCH, 1);

		input_report_abs(input_dev, ABS_X, x);
		input_report_abs(input_dev, ABS_Y, y);
		input_report_abs(input_dev, ABS_PRESSURE, Rt);
		input_sync(input_dev);
		return 0;
	}

	return -EINVAL;
}

static void ad7879_work(struct work_struct *work)
{
	struct ad7879 *ts = container_of(work, struct ad7879, work);

	/* use keventd context to read the result registers */
	ad7879_multi_read(ts, AD7879_REG_XPLUS, AD7879_NR_SENSE, ts->conversion_data);
	if (!ad7879_report(ts))
		mod_timer(&ts->timer, jiffies + TS_PEN_UP_TIMEOUT);
}

static void ad7879_ts_event_release(struct ad7879 *ts)
{
	struct input_dev *input_dev = ts->input;

	input_report_abs(input_dev, ABS_PRESSURE, 0);
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);
}

static void ad7879_timer(unsigned long handle)
{
	struct ad7879 *ts = (void *)handle;

	ad7879_ts_event_release(ts);
}

static irqreturn_t ad7879_irq(int irq, void *handle)
{
	struct ad7879 *ts = handle;

	/* The repeated conversion sequencer controlled by TMR kicked off too fast.
	 * We ignore the last and process the sample sequence currently in the queue.
	 * It can't be older than 9.4ms
	 */

	if (!work_pending(&ts->work))
		schedule_work(&ts->work);

	return IRQ_HANDLED;
}

static void ad7879_setup(struct ad7879 *ts)
{
	ad7879_write(ts, AD7879_REG_CTRL2, ts->cmd_crtl2);
	ad7879_write(ts, AD7879_REG_CTRL3, ts->cmd_crtl3);
	ad7879_write(ts, AD7879_REG_CTRL1, ts->cmd_crtl1);
}

int ad7879_disable(struct device *dev)
{
	struct ad7879 *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->mutex);

	if (!ts->disabled) {

		ts->disabled = 1;
		disable_irq(ts->bdata.irq);

		cancel_work_sync(&ts->work);

		if (del_timer_sync(&ts->timer))
			ad7879_ts_event_release(ts);

		ad7879_write(ts, AD7879_REG_CTRL2,
			     AD7879_PM(AD7879_PM_SHUTDOWN));
	}

	mutex_unlock(&ts->mutex);

	return 0;
}
EXPORT_SYMBOL(ad7879_disable);

int ad7879_enable(struct device *dev)
{
	struct ad7879 *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->mutex);

	if (ts->disabled) {
		ad7879_setup(ts);
		ts->disabled = 0;
		enable_irq(ts->bdata.irq);
	}

	mutex_unlock(&ts->mutex);

	return 0;
}
EXPORT_SYMBOL(ad7879_enable);

static ssize_t ad7879_disable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct ad7879 *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ts->disabled);
}

static ssize_t ad7879_disable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	if (val)
		ad7879_disable(dev);
	else
		ad7879_enable(dev);

	return count;
}

static DEVICE_ATTR(disable, 0664, ad7879_disable_show, ad7879_disable_store);

static struct attribute *ad7879_attributes[] = {
	&dev_attr_disable.attr,
	NULL
};

static const struct attribute_group ad7879_attr_group = {
	.attrs = ad7879_attributes,
};

#ifdef CONFIG_GPIOLIB
static int ad7879_gpio_direction_input(struct gpio_chip *chip,
					unsigned gpio)
{
	struct ad7879 *ts = container_of(chip, struct ad7879, gc);
	int err;

	mutex_lock(&ts->mutex);
	ts->cmd_crtl2 |= AD7879_GPIO_EN | AD7879_GPIODIR | AD7879_GPIOPOL;
	err = ad7879_write(ts, AD7879_REG_CTRL2, ts->cmd_crtl2);
	mutex_unlock(&ts->mutex);

	return err;
}

static int ad7879_gpio_direction_output(struct gpio_chip *chip,
					unsigned gpio, int level)
{
	struct ad7879 *ts = container_of(chip, struct ad7879, gc);
	int err;

	mutex_lock(&ts->mutex);
	ts->cmd_crtl2 &= ~AD7879_GPIODIR;
	ts->cmd_crtl2 |= AD7879_GPIO_EN | AD7879_GPIOPOL;
	if (level)
		ts->cmd_crtl2 |= AD7879_GPIO_DATA;
	else
		ts->cmd_crtl2 &= ~AD7879_GPIO_DATA;

	err = ad7879_write(ts, AD7879_REG_CTRL2, ts->cmd_crtl2);
	mutex_unlock(&ts->mutex);

	return err;
}

static int ad7879_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	struct ad7879 *ts = container_of(chip, struct ad7879, gc);
	u16 val;

	mutex_lock(&ts->mutex);
	val = ad7879_read(ts, AD7879_REG_CTRL2);
	mutex_unlock(&ts->mutex);

	return !!(val & AD7879_GPIO_DATA);
}

static void ad7879_gpio_set_value(struct gpio_chip *chip,
				  unsigned gpio, int value)
{
	struct ad7879 *ts = container_of(chip, struct ad7879, gc);

	mutex_lock(&ts->mutex);
	if (value)
		ts->cmd_crtl2 |= AD7879_GPIO_DATA;
	else
		ts->cmd_crtl2 &= ~AD7879_GPIO_DATA;

	ad7879_write(ts, AD7879_REG_CTRL2, ts->cmd_crtl2);
	mutex_unlock(&ts->mutex);
}

static int __devinit ad7879_gpio_add(struct device *dev)
{
	struct ad7879 *ts = dev_get_drvdata(dev);
	struct ad7879_platform_data *pdata = dev->platform_data;
	int ret = 0;

	if (pdata->gpio_export) {
		ts->gc.direction_input = ad7879_gpio_direction_input;
		ts->gc.direction_output = ad7879_gpio_direction_output;
		ts->gc.get = ad7879_gpio_get_value;
		ts->gc.set = ad7879_gpio_set_value;
		ts->gc.can_sleep = 1;
		ts->gc.base = pdata->gpio_base;
		ts->gc.ngpio = 1;
		ts->gc.label = "AD7879-GPIO";
		ts->gc.owner = THIS_MODULE;
		ts->gc.dev = dev;

		ret = gpiochip_add(&ts->gc);
		if (ret)
			dev_err(dev, "failed to register gpio %d\n",
				ts->gc.base);
	}

	return ret;
}

/*
 * We mark ad7879_gpio_remove inline so there is a chance the code
 * gets discarded when not needed. We can't do __devinit/__devexit
 * markup since it is used in both probe and remove methods.
 */
static inline void ad7879_gpio_remove(struct device *dev)
{
	struct ad7879 *ts = dev_get_drvdata(dev);
	struct ad7879_platform_data *pdata = dev->platform_data;
	int ret;

	if (pdata->gpio_export) {
		ret = gpiochip_remove(&ts->gc);
		if (ret)
			dev_err(dev, "failed to remove gpio %d\n",
				ts->gc.base);
	}
}
#else
static inline int ad7879_gpio_add(struct device *dev)
{
	return 0;
}

static inline void ad7879_gpio_remove(struct device *dev)
{
}
#endif

__devinit int
ad7879_probe(struct device *dev, struct ad7879_bus_data *bdata, u8 devid, u16 bustype)
{
	struct input_dev *input_dev;
	struct ad7879_platform_data *pdata = dev->platform_data;
	int err;
	u16 revid;
	struct ad7879 *ts;

	if (!bdata->irq) {
		dev_err(dev, "no IRQ?\n");
		return -ENODEV;
	}

	if (!pdata) {
		dev_err(dev, "no platform data?\n");
		return -ENODEV;
	}

	err = -ENOMEM;

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts)
		goto err_free_ts_mem;

	input_dev = input_allocate_device();
	if (!input_dev)
		goto err_free_ts_mem;

	ts->bdata = *bdata;
	ts->input = input_dev;
	dev_set_drvdata(dev, ts);

	setup_timer(&ts->timer, ad7879_timer, (unsigned long) ts);
	INIT_WORK(&ts->work, ad7879_work);
	mutex_init(&ts->mutex);

	ts->x_plate_ohms = pdata->x_plate_ohms ? : 400;
	ts->pressure_max = pdata->pressure_max ? : ~0;

	ts->first_conversion_delay = pdata->first_conversion_delay;
	ts->acquisition_time = pdata->acquisition_time;
	ts->averaging = pdata->averaging;
	ts->pen_down_acc_interval = pdata->pen_down_acc_interval;
	ts->median = pdata->median;

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(dev));

	input_dev->name = "AD7879 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = dev;
	input_dev->id.bustype = bustype;

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_PRESSURE, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_X,
			pdata->x_min ? : 0,
			pdata->x_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			pdata->y_min ? : 0,
			pdata->y_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			pdata->pressure_min, pdata->pressure_max, 0, 0);

	err = ad7879_write(ts, AD7879_REG_CTRL2, AD7879_RESET);

	if (err < 0) {
		dev_err(dev, "Failed to write %s\n", input_dev->name);
		goto err_free_mem;
	}

	revid = ad7879_read(ts, AD7879_REG_REVID);
	input_dev->id.product = (revid & 0xff);
	input_dev->id.version = revid >> 8;
	if (input_dev->id.product != devid) {
		dev_err(dev, "Failed to probe %s (%x vs %x)\n",
			input_dev->name, devid, revid);
		err = -ENODEV;
		goto err_free_mem;
	}

	ts->cmd_crtl3 = AD7879_YPLUS_BIT |
			AD7879_XPLUS_BIT |
			AD7879_Z2_BIT |
			AD7879_Z1_BIT |
			AD7879_TEMPMASK_BIT |
			AD7879_AUXVBATMASK_BIT |
			AD7879_GPIOALERTMASK_BIT;

	ts->cmd_crtl2 = AD7879_PM(AD7879_PM_DYN) | AD7879_DFR |
			AD7879_AVG(ts->averaging) |
			AD7879_MFS(ts->median) |
			AD7879_FCD(ts->first_conversion_delay);

	ts->cmd_crtl1 = AD7879_MODE_INT | AD7879_MODE_SEQ1 |
			AD7879_ACQ(ts->acquisition_time) |
			AD7879_TMR(ts->pen_down_acc_interval);

	ad7879_setup(ts);

	err = request_irq(ts->bdata.irq, ad7879_irq, IRQF_TRIGGER_FALLING,
			  dev_name(dev), ts);
	if (err) {
		dev_err(dev, "irq %d busy?\n", ts->bdata.irq);
		goto err_free_mem;
	}

	err = sysfs_create_group(&dev->kobj, &ad7879_attr_group);
	if (err)
		goto err_free_irq;

	err = ad7879_gpio_add(dev);
	if (err)
		goto err_remove_attr;

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_gpio;

	dev_info(dev, "Rev.%d touchscreen, irq %d\n",
		 revid >> 8, ts->bdata.irq);

	return 0;

err_remove_gpio:
	ad7879_gpio_remove(dev);
err_remove_attr:
	sysfs_remove_group(&dev->kobj, &ad7879_attr_group);
err_free_irq:
	free_irq(ts->bdata.irq, ts);
err_free_mem:
	input_free_device(input_dev);
err_free_ts_mem:
	kfree(ts);
	dev_set_drvdata(dev, NULL);

	return err;
}
EXPORT_SYMBOL(ad7879_probe);

__devexit int ad7879_remove(struct device *dev)
{
	struct ad7879 *ts = dev_get_drvdata(dev);

	ad7879_gpio_remove(dev);
	ad7879_disable(dev);
	sysfs_remove_group(&dev->kobj, &ad7879_attr_group);
	free_irq(ts->bdata.irq, ts);
	input_unregister_device(ts->input);
	dev_set_drvdata(dev, NULL);
	kfree(ts);

	dev_dbg(dev, "unregistered touchscreen\n");

	return 0;
}
EXPORT_SYMBOL(ad7879_remove);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("AD7879(-1) touchscreen Driver");
MODULE_LICENSE("GPL");
