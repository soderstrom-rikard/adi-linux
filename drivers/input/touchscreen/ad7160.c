/*
 * AD7160 based touchscreen
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Copyright (C) 2010 Michael Hennerich, Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <asm/unaligned.h>

#include <linux/input/ad7160.h>
#include "ad7160.h"

#define AD7160_SIL_ID			0x7160
#define AD7160_DRIVER_INTERFACE_ID	0x800

/*
 * Bit definition
 */

/* INT_GEST_EN_CTRL */
#define AD7160_TAP_ENABLE			(1 << 5)
#define AD7160_DBL_TAP_ENABLE			(1 << 6)
#define AD7160_EOC_HINT_ENABLE			(1 << 30)
#define AD7160_POS_HINT_ENABLE			(1 << 31)

/* GEST_STAT */
#define AD7160_TAP				(1 << 8)
#define AD7160_DOUBLE_TAP			(1 << 9)

/* LPM_CTRL */
#define AD7160_PWR_SAVE_CONV			(1 << 26)
#define AD7160_PROX_WK_UP			(1 << 28)
#define AD7160_FINGER_ACT_WK_UP			(1 << 29)
#define AD7160_LPM_LIFT_OFF_EN			(1 << 30)
#define AD7160_SHUTDOWN				(1 << 31)

/* DEMO_CTRL_STAT */
#define AD7160_SLF_OFFS_ADJUST			(1 << 0)
#define AD7160_MTL_OFFS_ADJUST			(1 << 1)
#define AD7160_SW_IIR_FILTER_EN			(1 << 2)
#define AD7160_EASY_SETUP			(1 << 3)
#define AD7160_MTL_OFFS_ADJUST_BUSY		(1 << 30)
#define AD7160_SLF_OFFS_ADJUST_BUSY		(1 << 31)


/* REV_ID */
#define AD7160_INTERFACE_REV_ID_SHIFT		0
#define AD7160_SILICON_REV_ID_SHIFT		16
#define AD7160_INTERFACE_REV_ID_MASK		0xFFFF
#define AD7160_SILICON_REV_ID_MASK		0xFFFF

#define AD7160_STAT_X_OFFS			0
#define AD7160_STAT_X_MASK			0xFFF
#define AD7160_STAT_Y_OFFS			12
#define AD7160_STAT_Y_MASK			0xFFF
#define AD7160_STAT_ID_OFFS			24
#define AD7160_STAT_ID_MASK			0xF
#define AD7160_STAT_ACT_OFFS			31
#define AD7160_STAT_ACT_MASK			0x1
#define AD7160_STAT_TMAJ_OFFS			0
#define AD7160_STAT_TMAJ_MASK			0x1FFF
#define AD7160_STAT_TMIN_OFFS			13
#define AD7160_STAT_TMIN_MASK			0x1FFF

#define AD7160_MAX_TRACKING_ID			0x0FFFF
#define AD7160_TRACKING_ID_FREE			0xF0000

struct ad7160 {
	struct ad7160_bus_data	bdata;
	struct input_dev	*input;
	struct work_struct	work;
	struct mutex		mutex;
	struct ad7160_platform_data *pdata;
	unsigned		disabled:1;	/* P: mutex */
	unsigned		handle_gest:1;

	u32			num_fingers;
	u32			gest_stat;
	u32			finger_data[MAX_NUM_FINGERS * 2];
	u32			tracking_id;
	u32			tracking_lut[MAX_NUM_FINGERS];
	u32			event_cabs;
	char			phys[32];
};

static int ad7160_read(struct ad7160 *ts, u32 reg)
{
	return ts->bdata.bops->read(ts->bdata.client, reg);
}
static int ad7160_multi_read(struct ad7160 *ts,
	u32 first_reg, u32 count, u32 *buf)
{
	return ts->bdata.bops->multi_read(ts->bdata.client,
		first_reg, count, buf);
}
static int ad7160_write(struct ad7160 *ts, u32 reg, u32 val)
{
	return ts->bdata.bops->write(ts->bdata.client, reg, val);
}

static unsigned ad7160_handle_tracking_id(struct ad7160 *ts,
		unsigned id, unsigned act)
{
	unsigned ret;

	if (ts->event_cabs & AD7160_TRACKING_ID_ASCENDING) {
		id--; /* ID is reported as 1..10 */
		if (act) {
			if (ts->tracking_lut[id] == AD7160_TRACKING_ID_FREE) {
				if (++ts->tracking_id > AD7160_MAX_TRACKING_ID)
					ts->tracking_id = 1;
				ts->tracking_lut[id] = ts->tracking_id;
			}

			return ts->tracking_lut[id];

		} else {
			if (ts->tracking_lut[id] == AD7160_TRACKING_ID_FREE)
				dev_err(ts->input->dev.parent,
					"Tracking ID table mismatch\n");

			ret = ts->tracking_lut[id];
			ts->tracking_lut[id] = AD7160_TRACKING_ID_FREE;
			return ret;
		}
	}
	return id;
}

static void ad7160_report_fingers(struct ad7160 *ts)
{
	struct input_dev *input_dev = ts->input;
	u32 x, y, a, id, maj, min, i;

	for (i = 0; i < (ts->num_fingers * 2); i += 2) {

		x = (ts->finger_data[i] >> AD7160_STAT_X_OFFS) &
			AD7160_STAT_X_MASK;
		y = (ts->finger_data[i] >> AD7160_STAT_Y_OFFS) &
			AD7160_STAT_Y_MASK;
		a = (ts->finger_data[i] >> AD7160_STAT_ACT_OFFS) &
			AD7160_STAT_ACT_MASK;
		id = (ts->finger_data[i] >> AD7160_STAT_ID_OFFS) &
			AD7160_STAT_ID_MASK;

#ifdef AD7160_REPORTS_TOUCH_MAJOR
		maj = (ts->finger_data[i + 1] >> AD7160_STAT_TMAJ_OFFS) &
			AD7160_STAT_TMAJ_MASK;
		min = (ts->finger_data[i + 1] >> AD7160_STAT_TMIN_OFFS) &
			AD7160_STAT_TMIN_MASK;
#else
		maj = min = a*10;
#endif

		if ((ts->event_cabs & AD7160_TRADITIONAL_TS_EMULATION) && (i == 0)) {
			input_report_abs(input_dev, ABS_X, x);
			input_report_abs(input_dev, ABS_Y, y);
			input_report_abs(input_dev, ABS_PRESSURE, a*10);
			input_report_key(input_dev, BTN_TOUCH, a);
			input_report_key(input_dev, BTN_TOOL_FINGER, 1);
		}

#ifdef ABS_MT_POSITION_X
		if (ts->event_cabs & AD7160_EMIT_ABS_MT_TRACKING_ID) {
			if (id < 1 || id > MAX_NUM_FINGERS)
				dev_err(ts->input->dev.parent,
					"invalid tracking ID (%d)\n", id);

			input_report_abs(input_dev, ABS_MT_TRACKING_ID,
				ad7160_handle_tracking_id(ts, id, a));
		} else {
			if (!a) {
				input_mt_sync(input_dev);
				continue;
			}
		}

		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);

		if (ts->event_cabs & AD7160_EMIT_ABS_MT_TOUCH_MAJOR)
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, maj);

		if (ts->event_cabs & AD7160_EMIT_ABS_MT_TOUCH_MINOR)
			input_report_abs(input_dev, ABS_MT_TOUCH_MINOR, min);

		if (ts->event_cabs & AD7160_EMIT_ABS_MT_ORIENTATION)
			input_report_abs(input_dev, ABS_MT_ORIENTATION, maj > min ? 1 : 0);

		if (ts->event_cabs & AD7160_EMIT_ABS_MT_PRESSURE)
			input_event(input_dev, EV_ABS, ABS_MT_PRESSURE, a);

		input_mt_sync(input_dev);
#endif
	}
}

static void ad7160_report_key_single(struct input_dev *input, int key)
{
	input_report_key(input, key, true);
	input_sync(input);
	input_report_key(input, key, false);
}

static void ad7160_report_gestures(struct ad7160 *ts)
{
	if (ts->gest_stat & AD7160_TAP)
		ad7160_report_key_single(ts->input,
			ts->pdata->ev_code_tap);

	if (ts->gest_stat & AD7160_DOUBLE_TAP)
		ad7160_report_key_single(ts->input,
			ts->pdata->ev_code_double_tap);
}

static void ad7160_report(struct ad7160 *ts)
{
	if (ts->num_fingers > 0)
		ad7160_report_fingers(ts);

	if (ts->gest_stat)
		ad7160_report_gestures(ts);

	input_sync(ts->input);
}

static void ad7160_work(struct work_struct *work)
{
	struct ad7160 *ts = container_of(work, struct ad7160, work);

	if (ts->handle_gest) {
		u32 dat[2];
		ad7160_multi_read(ts, AD7160_REG_GEST_STAT, 2, dat);
		ts->gest_stat = dat[0];		/* AD7160_REG_GEST_STAT */
		ts->num_fingers = dat[1];	/* AD7160_REG_NB_FINGERS */
	} else {
		ts->num_fingers = ad7160_read(ts, AD7160_REG_NB_FINGERS);
	}

#ifndef ABS_MT_POSITION_X
	/* For none MT enabled kernels we care only for the oldest contact */
	if (ts->num_fingers > 0)
		ts->num_fingers = 1;
#endif

	if ((ts->num_fingers > 0) && (ts->num_fingers <= MAX_NUM_FINGERS)) {
		ad7160_multi_read(ts, AD7160_REG_POS_DATA_STATUS1,
			ts->num_fingers * 2, ts->finger_data);
	} else {
		if (!ts->gest_stat)
			dev_err(ts->input->dev.parent,
				"invalid number of fingers (%d)\n",
				ts->num_fingers);
		ts->num_fingers = 0;
	}

	ad7160_report(ts);
	ad7160_feed_raw();
}

static irqreturn_t ad7160_irq(int irq, void *handle)
{
	struct ad7160 *ts = handle;
	if (!work_pending(&ts->work))
		schedule_work(&ts->work);

	return IRQ_HANDLED;
}

static void ad7160_setup(struct ad7160 *ts)
{
	int i;

	for (i = 0; i < MAX_NUM_FINGERS; i++)
		ts->tracking_lut[i] = AD7160_TRACKING_ID_FREE;

	ts->tracking_id = 1;

	ad7160_write(ts, AD7160_REG_LPM_CTRL, 0);
	ad7160_write(ts, AD7160_REG_INT_GEST_EN_CTRL,
			(ts->pdata->ev_code_tap != 0 ? AD7160_TAP_ENABLE : 0) |
			(ts->pdata->ev_code_double_tap != 0
			? AD7160_DBL_TAP_ENABLE : 0) |
			AD7160_POS_HINT_ENABLE);

	/* Sensor resolution */
	ad7160_write(ts, AD7160_REG_XY_RES,
			(ts->pdata->coord_pref << 28) |
			(ts->pdata->filter_coef << 24) |
			(ts->pdata->sensor_y_res << 12) |
			ts->pdata->sensor_x_res);

	/* Reset demo control/status register */
	ad7160_write(ts, AD7160_REG_DEMO_CTRL_STAT, AD7160_SW_IIR_FILTER_EN);

	/* Position window update control register */
	ad7160_write(ts, AD7160_REG_POSITION_WINDOW_CTRL,
		(ts->pdata->move_window << 8) |
		ts->pdata->first_touch_window);

	ad7160_write(ts, AD7160_REG_FINGER_ACT_CTRL,
		ts->pdata->finger_act_ctrl);
}

void ad7160_disable(struct device *dev)
{
	struct ad7160 *ts = dev_get_drvdata(dev);
	mutex_lock(&ts->mutex);

	if (!ts->disabled) {
		ad7160_write(ts, AD7160_REG_LPM_CTRL, AD7160_SHUTDOWN);
		ts->disabled = 1;
		disable_irq(ts->bdata.irq);
		cancel_work_sync(&ts->work);
	}

	mutex_unlock(&ts->mutex);
}
EXPORT_SYMBOL(ad7160_disable);

void ad7160_enable(struct device *dev)
{
	struct ad7160 *ts = dev_get_drvdata(dev);
	mutex_lock(&ts->mutex);

	if (ts->disabled) {
		ad7160_setup(ts);
		ts->disabled = 0;
		enable_irq(ts->bdata.irq);
	}

	mutex_unlock(&ts->mutex);
}
EXPORT_SYMBOL(ad7160_enable);

static ssize_t ad7160_disable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct ad7160 *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ts->disabled);
}

static ssize_t ad7160_disable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	if (val)
		ad7160_disable(dev);
	else
		ad7160_enable(dev);

	return count;
}

static DEVICE_ATTR(disable, 0664, ad7160_disable_show, ad7160_disable_store);

static struct attribute *ad7160_attributes[] = {
	&dev_attr_disable.attr,
	NULL
};

static const struct attribute_group ad7160_attr_group = {
	.attrs = ad7160_attributes,
};

__devinit int
ad7160_probe(struct device *dev, struct ad7160_bus_data *bdata,
		u32 devid, u16 bustype)
{
	struct input_dev *input_dev;
	struct ad7160_platform_data *pdata = dev->platform_data;
	int err;
	u32 revid, fw_revid;
	struct ad7160 *ts;

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
	ts->pdata = pdata;
	ts->input = input_dev;
	dev_set_drvdata(dev, ts);

	ts->event_cabs = pdata->event_cabs;

	INIT_WORK(&ts->work, ad7160_work);
	mutex_init(&ts->mutex);

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(dev));

	input_dev->name = "AD7160 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = dev;
	input_dev->id.bustype = bustype;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);

	if (pdata->ev_code_tap) {
		__set_bit(pdata->ev_code_tap, input_dev->keybit);
		ts->handle_gest = true;
	}
	if (pdata->ev_code_double_tap) {
		__set_bit(pdata->ev_code_double_tap, input_dev->keybit);
		ts->handle_gest = true;
	}

	if (ts->event_cabs & AD7160_TRADITIONAL_TS_EMULATION) {

		__set_bit(ABS_X, input_dev->absbit);
		__set_bit(ABS_Y, input_dev->absbit);
		__set_bit(ABS_PRESSURE, input_dev->absbit);

		input_set_abs_params(input_dev, ABS_X, 0,
				pdata->sensor_x_res, 0, 0);
		input_set_abs_params(input_dev, ABS_Y, 0,
				pdata->sensor_y_res, 0, 0);
		input_set_abs_params(input_dev, ABS_PRESSURE, 0,
				pdata->pressure, 0, 0);

		__set_bit(BTN_TOUCH, input_dev->keybit);
		__set_bit(BTN_TOOL_FINGER, input_dev->keybit);
	}

#ifdef ABS_MT_POSITION_X
	if (ts->event_cabs & AD7160_EMIT_ABS_MT_TRACKING_ID)
		input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
			1, AD7160_MAX_TRACKING_ID, 0, 0);

	if (ts->event_cabs & AD7160_EMIT_ABS_MT_PRESSURE)
		input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 100, 0, 0);

	if (ts->event_cabs & AD7160_EMIT_ABS_MT_TOUCH_MAJOR) {
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0,
			AD7160_STAT_TMAJ_MASK, 0, 0);
	}

	if (ts->event_cabs & AD7160_EMIT_ABS_MT_TOUCH_MINOR) {
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0,
			AD7160_STAT_TMIN_MASK, 0, 0);
	}

	if (ts->event_cabs & AD7160_EMIT_ABS_MT_ORIENTATION)
		input_set_abs_params(input_dev, ABS_MT_ORIENTATION, 0, 1, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
		ts->pdata->sensor_x_res, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
		ts->pdata->sensor_y_res, 0, 0);

#endif
	input_dev->id.product = ad7160_read(ts, AD7160_REG_DEVICE_ID) & 0xFFFF;

	if (input_dev->id.product != AD7160_SIL_ID) {
		dev_err(dev, "Failed to probe %s (%x vs %x)\n",
			input_dev->name, input_dev->id.product, AD7160_SIL_ID);
		err = -ENODEV;
		goto err_free_mem;
	}

	fw_revid = ad7160_read(ts, AD7160_REG_FW_REV);
	revid = ad7160_read(ts, AD7160_REG_REV_ID);

	input_dev->id.version = (revid >> AD7160_INTERFACE_REV_ID_SHIFT) &
				AD7160_INTERFACE_REV_ID_MASK;

	ad7160_setup(ts);

	err = request_irq(ts->bdata.irq, ad7160_irq, IRQF_TRIGGER_FALLING,
			  dev_name(dev), ts);
	if (err) {
		dev_err(dev, "irq %d busy?\n", ts->bdata.irq);
		goto err_free_mem;
	}

	err = sysfs_create_group(&dev->kobj, &ad7160_attr_group);
	if (err)
		goto err_free_irq;

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_attr;

	if (input_dev->id.version > AD7160_DRIVER_INTERFACE_ID)
		dev_warn(dev, "Driver implements Interface REV %x, "
		"device probed implements %x!\n", AD7160_DRIVER_INTERFACE_ID,
		input_dev->id.version);

	dev_info(dev, "Rev.%d touchscreen, irq %d, Interface REV %x, "
		 "Firmware REV %d.%d.%d.%d\n",
		 (revid >> AD7160_SILICON_REV_ID_SHIFT) &
		 AD7160_SILICON_REV_ID_MASK,
		 ts->bdata.irq, input_dev->id.version, fw_revid >> 12,
		 (fw_revid >> 8) & 0xF, (fw_revid >> 4) & 0xF, fw_revid & 0xF);

	return 0;

err_remove_attr:
	sysfs_remove_group(&dev->kobj, &ad7160_attr_group);
err_free_irq:
	free_irq(ts->bdata.irq, ts);
err_free_mem:
	input_free_device(input_dev);
err_free_ts_mem:
	kfree(ts);
	dev_set_drvdata(dev, NULL);

	return err;
}
EXPORT_SYMBOL(ad7160_probe);

__devexit int ad7160_remove(struct device *dev)
{
	struct ad7160 *ts = dev_get_drvdata(dev);

	ad7160_disable(dev);
	sysfs_remove_group(&dev->kobj, &ad7160_attr_group);
	free_irq(ts->bdata.irq, ts);
	input_unregister_device(ts->input);
	dev_set_drvdata(dev, NULL);
	kfree(ts);

	dev_dbg(dev, "unregistered touchscreen\n");

	return 0;
}
EXPORT_SYMBOL(ad7160_remove);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("AD7160 Touchscreen Driver");
MODULE_LICENSE("GPL");
