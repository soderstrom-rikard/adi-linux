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
#include <linux/hrtimer.h>
#include <asm/unaligned.h>
#include <linux/firmware.h>
#include <linux/delay.h>

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

#define AD7160_LPM_TIMEOUTms(x)			((x) & 0xFFFF)
#define AD7160_LPM_WAKEUP_INTERVALms(x)		(((x) & 0x3FF) << 16)
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

/* HAPTIC_CTRL */
#define AD7160_HAPTIC_EFFECTS(x)		((x) & 0xF)
#define AD7160_HAPTIC_EN			(1 << 4)
#define AD7160_ACTUATOR_EN			(1 << 5)

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
#define AD7160_MAX_TIMED_HAPTIC_DUR		100000

/* Firmware Loader Defines */

#define CMD_ERASE_PAGE		0xC1
#define CMD_DWNLD_DATA		0xC2
#define CMD_VERIFY		0xC3
#define AD7160_NUM_OF_PAGES	62
#define AD7160_PAGE_SIZE	512
#define AD7160_MAX_FW_SIZE	(AD7160_NUM_OF_PAGES * AD7160_PAGE_SIZE)
#define AD7160_CMD_HDR_SIZE	5
#define AD7160_SPLIT_SIZE	200

#define BOOT_MODE_MAGIC		0x012357BD
#define BOOT_MODE_REVID		0x10AD
#define NORM_MODE_REVID		0x7160
#define MODE_REVID_SHIFT	16

#define FIRMWARE		"ad7160_fw.bin"
#define AD7160_FW_MAGIC		0x00AD7160

struct fw_header {
	unsigned int fmagic;
	unsigned int hlenght;
	unsigned int crc24;
} __attribute__ ((packed));

/*
 * CRC-24 Calculation using a look up table specific to
 * the polynome: x^24 + x^23 + x^6 + x^5 + x + 1
 */

/* CRC24 definitions */
#define CRC_INIT	0xFFFFFFFF
#define CRC_WIDTH	24
#define CRC_MASK	((1<<(CRC_WIDTH)) - 1)
#define CRC_FINAL_XOR	0x00000000

struct ad7160 {
	struct ad7160_bus_data	bdata;
	struct input_dev	*input;
	struct work_struct	work;
	struct mutex		mutex;
	struct ad7160_platform_data *pdata;

	struct hrtimer		timer;
	struct work_struct	hwork;
	struct mutex		hmutex;
	bool			disabled;	/* P: mutex */
	bool			opened;		/* P: mutex */
	bool			suspended;	/* P: mutex */
	bool			irq_disabled;	/* P: mutex */

	unsigned		handle_gest:1;
	unsigned		lpm_ctrl;

	u32			num_fingers;
	u32			gest_stat;
	u32			finger_data[MAX_NUM_FINGERS * 2];
	u32			tracking_id;
	u32			tracking_lut[MAX_NUM_FINGERS];
	u32			event_cabs;
	char			phys[32];
};

static inline int ad7160_read(struct ad7160 *ts, u32 reg)
{
	return ts->bdata.bops->read(ts->bdata.client, reg);
}
static inline int ad7160_multi_read(struct ad7160 *ts,
	u32 first_reg, u32 count, u32 *buf)
{
	return ts->bdata.bops->multi_read(ts->bdata.client,
		first_reg, count, buf);
}
static inline int ad7160_write(struct ad7160 *ts, u32 reg, u32 val)
{
	return ts->bdata.bops->write(ts->bdata.client, reg, val);
}

static inline int ad7160_write_bytes(struct ad7160 *ts, u32 count, u8 *buf)
{
	return ts->bdata.bops->multi_write_bytes(ts->bdata.client,
		count, buf);
}

static inline void ad7160_wakeup(struct ad7160 *ts)
{
	ts->bdata.bops->wakeup(ts->bdata.client);
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
	u32 *effect_ctl;

	for (i = 0; i < MAX_NUM_FINGERS; i++)
		ts->tracking_lut[i] = AD7160_TRACKING_ID_FREE;

	ts->tracking_id = 1;

	for (i = AD7160_REG_HAPTIC_EFFECT1_CTRL,
	     effect_ctl = &ts->pdata->haptic_effect1_ctrl;
	     i <= AD7160_REG_HAPTIC_EFFECT6_CTRL3; i += 4)
		ad7160_write(ts, i, *effect_ctl++);

	ts->lpm_ctrl = AD7160_LPM_TIMEOUTms(1000) |
			 AD7160_LPM_WAKEUP_INTERVALms(200) |
			 AD7160_PROX_WK_UP |
			 AD7160_FINGER_ACT_WK_UP;

	ad7160_write(ts, AD7160_REG_LPM_CTRL, ts->lpm_ctrl);
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

	/* Position window update control register */
	ad7160_write(ts, AD7160_REG_POSITION_WINDOW_CTRL,
		(ts->pdata->move_window << 8) |
		ts->pdata->first_touch_window);

	ad7160_write(ts, AD7160_REG_FINGER_ACT_CTRL,
		ts->pdata->finger_act_ctrl);
}

static void __ad7160_disable(struct ad7160 *ts)
{
	disable_irq_nosync(ts->bdata.irq);
	cancel_work_sync(&ts->work);
	ts->irq_disabled = true;
}

static void __ad7160_enable(struct ad7160 *ts)
{
	enable_irq(ts->bdata.irq);
	ts->irq_disabled = false;
}

static int ad7160_input_open(struct input_dev *input)
{
	struct ad7160 *ts = input_get_drvdata(input);

	mutex_lock(&ts->mutex);
	if (!ts->suspended && !ts->disabled)
		__ad7160_enable(ts);

	ts->opened = true;
	mutex_unlock(&ts->mutex);

	return 0;
}

static void ad7160_input_close(struct input_dev *input)
{
	struct ad7160 *ts = input_get_drvdata(input);

	mutex_lock(&ts->mutex);
	if (!ts->suspended && !ts->disabled)
		__ad7160_disable(ts);

	ts->opened = false;
	mutex_unlock(&ts->mutex);
}


void ad7160_suspend(struct device *dev)
{
	struct ad7160 *ts = dev_get_drvdata(dev);

	ad7160_write(ts, AD7160_REG_LPM_CTRL,
		     ts->lpm_ctrl | AD7160_SHUTDOWN);

	mutex_lock(&ts->mutex);
	if (!ts->suspended && !ts->disabled && ts->opened)
		__ad7160_disable(ts);

	ts->suspended = true;
	mutex_unlock(&ts->mutex);
}

void ad7160_resume(struct device *dev)
{
	struct ad7160 *ts = dev_get_drvdata(dev);

	ad7160_wakeup(ts);
	ad7160_write(ts, AD7160_REG_LPM_CTRL, ts->lpm_ctrl);
	ad7160_setup(ts);

	mutex_lock(&ts->mutex);
	if (ts->suspended && !ts->disabled && ts->opened)
		__ad7160_enable(ts);

	ts->suspended = false;
	mutex_unlock(&ts->mutex);
}

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
	struct ad7160 *ts = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	mutex_lock(&ts->mutex);

	if (!ts->suspended && ts->opened) {
		if (val) {
			if (!ts->disabled)
				__ad7160_disable(ts);
		} else {
			if (ts->disabled)
				__ad7160_enable(ts);
		}
	}

	ts->disabled = !!val;

	mutex_unlock(&ts->mutex);

	return count;
}

static DEVICE_ATTR(disable, 0664, ad7160_disable_show, ad7160_disable_store);

static void ad7160_hwork(struct work_struct *work)
{
	struct ad7160 *ts = container_of(work, struct ad7160, hwork);
	ad7160_write(ts, AD7160_REG_HAPTIC_CTRL, 0);
}

static enum hrtimer_restart ad7160_haptic_timer_func(struct hrtimer *timer)
{
	struct ad7160 *ts =
		container_of(timer, struct ad7160, timer);

	schedule_work(&ts->hwork);

	return HRTIMER_NORESTART;
}

static ssize_t ad7160_timed_haptic_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct ad7160 *ts = dev_get_drvdata(dev);
	int remaining;

	if (hrtimer_active(&ts->timer)) {
		ktime_t r = hrtimer_get_remaining(&ts->timer);
		struct timeval t = ktime_to_timeval(r);
		remaining = t.tv_sec * 1000 + t.tv_usec / 1000;
	} else
		remaining = 0;

	return sprintf(buf, "%d\n", remaining);
}

static ssize_t ad7160_timed_haptic_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct ad7160 *ts = dev_get_drvdata(dev);
	unsigned long value;
	int error;

	error = strict_strtoul(buf, 10, &value);
	if (error)
		return error;

	mutex_lock(&ts->hmutex);
	/* cancel previous timer and wait for the handler to finish.  */
	hrtimer_cancel(&ts->timer);

	if (value > 0) {
		if (value > AD7160_MAX_TIMED_HAPTIC_DUR)
			value = AD7160_MAX_TIMED_HAPTIC_DUR;

		ad7160_write(ts, AD7160_REG_HAPTIC_CTRL, AD7160_ACTUATOR_EN);

		hrtimer_start(&ts->timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	}

	mutex_unlock(&ts->hmutex);

	return count;
}

static DEVICE_ATTR(timed_haptic, 0664, ad7160_timed_haptic_show, ad7160_timed_haptic_store);

static ssize_t ad7160_effect_haptic_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct ad7160 *ts = dev_get_drvdata(dev);
	unsigned long value;
	int error;

	error = strict_strtoul(buf, 10, &value);
	if (error)
		return error;

	mutex_lock(&ts->hmutex);

	if ((value > 0) && (value < 16))
		ad7160_write(ts, AD7160_REG_HAPTIC_CTRL, AD7160_HAPTIC_EFFECTS(value) | AD7160_HAPTIC_EN);

	mutex_unlock(&ts->hmutex);

	return count;
}

static DEVICE_ATTR(effect_haptic, 0664, NULL, ad7160_effect_haptic_store);

#if defined(CONFIG_TOUCHSCREEN_AD7160_FW) || defined(CONFIG_TOUCHSCREEN_AD7160_FW_MODULE)
static unsigned int crc_table[256] = {
	0x00000000, 0x00800063, 0x008000A5, 0x000000C6,
	0x00800129, 0x0000014A, 0x0000018C, 0x008001EF,
	0x00800231, 0x00000252, 0x00000294, 0x008002F7,
	0x00000318, 0x0080037B, 0x008003BD, 0x000003DE,
	0x00800401, 0x00000462, 0x000004A4, 0x008004C7,
	0x00000528, 0x0080054B, 0x0080058D, 0x000005EE,
	0x00000630, 0x00800653, 0x00800695, 0x000006F6,
	0x00800719, 0x0000077A, 0x000007BC, 0x008007DF,
	0x00800861, 0x00000802, 0x000008C4, 0x008008A7,
	0x00000948, 0x0080092B, 0x008009ED, 0x0000098E,
	0x00000A50, 0x00800A33, 0x00800AF5, 0x00000A96,
	0x00800B79, 0x00000B1A, 0x00000BDC, 0x00800BBF,
	0x00000C60, 0x00800C03, 0x00800CC5, 0x00000CA6,
	0x00800D49, 0x00000D2A, 0x00000DEC, 0x00800D8F,
	0x00800E51, 0x00000E32, 0x00000EF4, 0x00800E97,
	0x00000F78, 0x00800F1B, 0x00800FDD, 0x00000FBE,
	0x008010A1, 0x000010C2, 0x00001004, 0x00801067,
	0x00001188, 0x008011EB, 0x0080112D, 0x0000114E,
	0x00001290, 0x008012F3, 0x00801235, 0x00001256,
	0x008013B9, 0x000013DA, 0x0000131C, 0x0080137F,
	0x000014A0, 0x008014C3, 0x00801405, 0x00001466,
	0x00801589, 0x000015EA, 0x0000152C, 0x0080154F,
	0x00801691, 0x000016F2, 0x00001634, 0x00801657,
	0x000017B8, 0x008017DB, 0x0080171D, 0x0000177E,
	0x000018C0, 0x008018A3, 0x00801865, 0x00001806,
	0x008019E9, 0x0000198A, 0x0000194C, 0x0080192F,
	0x00801AF1, 0x00001A92, 0x00001A54, 0x00801A37,
	0x00001BD8, 0x00801BBB, 0x00801B7D, 0x00001B1E,
	0x00801CC1, 0x00001CA2, 0x00001C64, 0x00801C07,
	0x00001DE8, 0x00801D8B, 0x00801D4D, 0x00001D2E,
	0x00001EF0, 0x00801E93, 0x00801E55, 0x00001E36,
	0x00801FD9, 0x00001FBA, 0x00001F7C, 0x00801F1F,
	0x00802121, 0x00002142, 0x00002184, 0x008021E7,
	0x00002008, 0x0080206B, 0x008020AD, 0x000020CE,
	0x00002310, 0x00802373, 0x008023B5, 0x000023D6,
	0x00802239, 0x0000225A, 0x0000229C, 0x008022FF,
	0x00002520, 0x00802543, 0x00802585, 0x000025E6,
	0x00802409, 0x0000246A, 0x000024AC, 0x008024CF,
	0x00802711, 0x00002772, 0x000027B4, 0x008027D7,
	0x00002638, 0x0080265B, 0x0080269D, 0x000026FE,
	0x00002940, 0x00802923, 0x008029E5, 0x00002986,
	0x00802869, 0x0000280A, 0x000028CC, 0x008028AF,
	0x00802B71, 0x00002B12, 0x00002BD4, 0x00802BB7,
	0x00002A58, 0x00802A3B, 0x00802AFD, 0x00002A9E,
	0x00802D41, 0x00002D22, 0x00002DE4, 0x00802D87,
	0x00002C68, 0x00802C0B, 0x00802CCD, 0x00002CAE,
	0x00002F70, 0x00802F13, 0x00802FD5, 0x00002FB6,
	0x00802E59, 0x00002E3A, 0x00002EFC, 0x00802E9F,
	0x00003180, 0x008031E3, 0x00803125, 0x00003146,
	0x008030A9, 0x000030CA, 0x0000300C, 0x0080306F,
	0x008033B1, 0x000033D2, 0x00003314, 0x00803377,
	0x00003298, 0x008032FB, 0x0080323D, 0x0000325E,
	0x00803581, 0x000035E2, 0x00003524, 0x00803547,
	0x000034A8, 0x008034CB, 0x0080340D, 0x0000346E,
	0x000037B0, 0x008037D3, 0x00803715, 0x00003776,
	0x00803699, 0x000036FA, 0x0000363C, 0x0080365F,
	0x008039E1, 0x00003982, 0x00003944, 0x00803927,
	0x000038C8, 0x008038AB, 0x0080386D, 0x0000380E,
	0x00003BD0, 0x00803BB3, 0x00803B75, 0x00003B16,
	0x00803AF9, 0x00003A9A, 0x00003A5C, 0x00803A3F,
	0x00003DE0, 0x00803D83, 0x00803D45, 0x00003D26,
	0x00803CC9, 0x00003CAA, 0x00003C6C, 0x00803C0F,
	0x00803FD1, 0x00003FB2, 0x00003F74, 0x00803F17,
	0x00003EF8, 0x00803E9B, 0x00803E5D, 0x00003E3E,
};

static unsigned int do_crc24(unsigned long len, unsigned char *data)
{
	unsigned int crc  = CRC_INIT;

#if defined(__BIG_ENDIAN)
	while (len--) {
		crc = crc_table[((crc >> (CRC_WIDTH-8)) ^ *data++) & 0xFFL]
			^ (crc << 8);
	}
	crc ^= CRC_FINAL_XOR;
#elif defined(__LITTLE_ENDIAN)
	len = len / 4;
	while (len--) {
		crc = crc_table[((crc >> (CRC_WIDTH-8)) ^ data[3]) & 0xFFL]
			^ (crc << 8);
		crc = crc_table[((crc >> (CRC_WIDTH-8)) ^ data[2]) & 0xFFL]
			^ (crc << 8);
		crc = crc_table[((crc >> (CRC_WIDTH-8)) ^ data[1]) & 0xFFL]
			^ (crc << 8);
		crc = crc_table[((crc >> (CRC_WIDTH-8)) ^ data[0]) & 0xFFL]
			^ (crc << 8);

		data += 4;
	}
	crc ^= CRC_FINAL_XOR;
#else
#error ("ENDIAN not recognized")
#endif
	return crc & CRC_MASK;
}

static int ad7160_enter_boot_mode(struct ad7160 *ts)
{
	int ret;
	unsigned id;

	ret = ad7160_write(ts, AD7160_REG_BOOT_MODE_CTRL,
			   BOOT_MODE_MAGIC);
	if (ret < 0)
		return ret;

	/* write BOOT_MODE_MAGIC twice to enter boot mode */
	ad7160_write(ts, AD7160_REG_BOOT_MODE_CTRL,
		     BOOT_MODE_MAGIC);

	mdelay(50);	/* Wait for 50ms */

	id = ad7160_read(ts, AD7160_REG_AFE_DEVID);
	if ((id >> MODE_REVID_SHIFT) == BOOT_MODE_REVID)
		return 0;

	return -EFAULT;
}

static int ad7160_flash_erase(struct ad7160 *ts)
{
	unsigned char cmd = CMD_ERASE_PAGE;
	int ret;

	ret = ad7160_write_bytes(ts, 1, &cmd);
	mdelay(2000);	/* Wait for 2secs */

	return ret;
}

static int ad7160_flash(struct ad7160 *ts, unsigned char *data,
			unsigned int size)
{
	int ret, i;
	unsigned offset;

	for (i = 0; i < DIV_ROUND_UP(size, AD7160_SPLIT_SIZE); i++) {
		offset = i * AD7160_SPLIT_SIZE;
		data[0 + offset] = CMD_DWNLD_DATA;		/* command */
		data[1 + offset] = offset >> 8;			/* offset msb */
		data[2 + offset] = offset & 0xFF;		/* offset lsb */
		data[3 + offset] = AD7160_SPLIT_SIZE >> 8;	/* buffer size msb */
		data[4 + offset] = AD7160_SPLIT_SIZE & 0xFF;	/* buffer size lsb */

		ret = ad7160_write_bytes(ts, AD7160_CMD_HDR_SIZE +
					 AD7160_SPLIT_SIZE,
					 &data[i * AD7160_SPLIT_SIZE]);
	}

	return ret;
}

static int ad7160_flash_verify(struct ad7160 *ts, unsigned int crc24)
{
	unsigned char cmd[9];
	unsigned id;

	memset(cmd, 0, sizeof(cmd));

	cmd[0] = CMD_VERIFY;
	cmd[6] = (crc24 >> 16) & 0xFF;
	cmd[7] = (crc24 >> 8) & 0xFF;
	cmd[8] = crc24 & 0xFF;

	ad7160_write_bytes(ts, sizeof(cmd), cmd);

	mdelay(1000);

	id = ad7160_read(ts, AD7160_REG_AFE_DEVID);
	if ((id >> MODE_REVID_SHIFT) == NORM_MODE_REVID)
		return 0;

	return -EFAULT;
}

static int ad7160_flash_firmware(struct device *dev, unsigned char *data,
				 unsigned int size, unsigned int crc24)
{
	struct ad7160 *ts = dev_get_drvdata(dev);
	int ret;
	bool irq_disabled;

	mutex_lock(&ts->mutex);
	irq_disabled = ts->irq_disabled;
	if (!irq_disabled)
		__ad7160_disable(ts);

	ret = ad7160_enter_boot_mode(ts);
	if (ret < 0) {
		dev_err(dev, "failed to enter boot mode\n");
		goto out_failed;
	}

	ret = ad7160_flash_erase(ts);
	if (ret < 0)			/* no way out - continue anyways */
		dev_err(dev, "failed to send erase command\n");

	ret = ad7160_flash(ts, data, size);
	if (ret < 0)			/* no way out - continue anyways */
		dev_err(dev, "failed to flash\n");

	ret = ad7160_flash_verify(ts, crc24);
	if (ret < 0)			/* no way out - continue anyways */
		dev_err(dev, "verify flash failed\n");

	ad7160_setup(ts);
out_failed:
	if (!irq_disabled)
		__ad7160_enable(ts);
	mutex_unlock(&ts->mutex);

	return ret;
}

static int ad7160_update_fw(struct device *dev, const char *fw_name)
{
	const struct firmware *fw;
	struct fw_header *header;
	int ret;
	unsigned char *temp;
	unsigned hlenght;

	ret = request_firmware(&fw, fw_name, dev);
	if (ret) {
		dev_err(dev, "request_firmware() failed with %i\n", ret);
		return ret;
	}

	header = (struct fw_header *) fw->data;

	if (le32_to_cpu(header->fmagic) != AD7160_FW_MAGIC) {
		dev_err(dev, "firmware file magic failed\n");
		ret = -EFAULT;
		goto out_release_fw;
	}

	hlenght = le32_to_cpu(header->hlenght);

	if ((fw->size - hlenght) > AD7160_MAX_FW_SIZE) {
		dev_err(dev, "invalid firmware size (%d)\n", (int)fw->size);
		ret = -EFAULT;
		goto out_release_fw;
	}

	temp = kmalloc(AD7160_MAX_FW_SIZE + AD7160_CMD_HDR_SIZE, GFP_KERNEL);
	if (temp == NULL) {
		ret = -ENOMEM;
		goto out_release_fw;
	}

	memcpy(temp + AD7160_CMD_HDR_SIZE, fw->data + hlenght, fw->size - hlenght);

	/* pad the remaining bytes with 0xFF */
	memset(temp + AD7160_CMD_HDR_SIZE + fw->size - hlenght, 0xFF,
	       AD7160_MAX_FW_SIZE - fw->size + hlenght);

	/*
	 * The upper 4 bytes of a block is not included when generating
	 * the crc/signature, because this is where the signature is stored.
	 */

	if (do_crc24(AD7160_MAX_FW_SIZE - 4, temp + AD7160_CMD_HDR_SIZE)
		!= le32_to_cpu(header->crc24)) {

		dev_err(dev, "firmware crc check failed\n");
		ret = -EFAULT;
		goto out_free_mem;
	}
	/* MAGIC and CRC ok */
	ret = ad7160_flash_firmware(dev, temp, AD7160_MAX_FW_SIZE,
				    le32_to_cpu(header->crc24));

out_free_mem:
	kfree(temp);
out_release_fw:
	release_firmware(fw);
	return ret;
}

static ssize_t ad7160_update_fw_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long value;
	int error;

	error = strict_strtoul(buf, 10, &value);
	if (error)
		return error;

	if (value == 1)
		ad7160_update_fw(dev, FIRMWARE);

	return count;
}

static DEVICE_ATTR(update_fw, 0664, NULL, ad7160_update_fw_store);
#endif /* CONFIG_TOUCHSCREEN_AD7160_FW */

static struct attribute *ad7160_attributes[] = {
	&dev_attr_disable.attr,
	&dev_attr_timed_haptic.attr,
	&dev_attr_effect_haptic.attr,
#if defined(CONFIG_TOUCHSCREEN_AD7160_FW) || defined(CONFIG_TOUCHSCREEN_AD7160_FW_MODULE)
	&dev_attr_update_fw.attr,
#endif
	NULL
};

static const struct attribute_group ad7160_attr_group = {
	.attrs = ad7160_attributes,
};

int
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
	input_dev->open = ad7160_input_open;
	input_dev->close = ad7160_input_close;

	input_set_drvdata(input_dev, ts);

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
		if ((ad7160_read(ts, AD7160_REG_AFE_DEVID) >> MODE_REVID_SHIFT)
			== BOOT_MODE_REVID) {
			dev_warn(dev, "Device in boot mode - upload firmware!\n");
		} else  {
			dev_err(dev, "Failed to probe %s (%x vs %x)\n",
				input_dev->name, input_dev->id.product, AD7160_SIL_ID);
			err = -ENODEV;
			goto err_free_mem;
		}
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

	disable_irq_nosync(ts->bdata.irq);

	INIT_WORK(&ts->hwork, ad7160_hwork);
	mutex_init(&ts->hmutex);
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = ad7160_haptic_timer_func;

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

int ad7160_remove(struct device *dev)
{
	struct ad7160 *ts = dev_get_drvdata(dev);

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
