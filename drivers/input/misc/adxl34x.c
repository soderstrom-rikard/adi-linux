/*
 * Copyright (C) 2009 Michael Hennerich, Analog Devices Inc.
 *
 * Description:	ADXL345/346 Three-Axis Digital Accelerometers (I2C/SPI Interface)
 *
 * Bugs:        Enter bugs at http://blackfin.uclinux.org/
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
 *
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>

#include <linux/spi/adxl34x.h>

/* ADXL345/6 Register Map */
#define DEVID		0x00	/* R   Device ID */
#define THRESH_TAP 	0x1D	/* R/W Tap threshold */
#define OFSX 		0x1E	/* R/W X-axis offset */
#define OFSY 		0x1F	/* R/W Y-axis offset */
#define OFSZ 		0x20	/* R/W Z-axis offset */
#define DUR 		0x21	/* R/W Tap duration */
#define LATENT 		0x22	/* R/W Tap latency */
#define WINDOW 		0x23	/* R/W Tap window */
#define THRESH_ACT 	0x24	/* R/W Activity threshold */
#define THRESH_INACT 	0x25	/* R/W Inactivity threshold */
#define TIME_INACT 	0x26	/* R/W Inactivity time */
#define ACT_INACT_CTL 	0x27	/* R/W Axis enable control for activity and inactivity detection */
#define THRESH_FF 	0x28	/* R/W Free-fall threshold */
#define TIME_FF 	0x29	/* R/W Free-fall time */
#define TAP_AXES 	0x2A	/* R/W Axis control for tap/double tap */
#define ACT_TAP_STATUS 	0x2B	/* R   Source of tap/double tap */
#define BW_RATE 	0x2C	/* R/W Data rate and power mode control */
#define POWER_CTL 	0x2D	/* R/W Power saving features control */
#define INT_ENABLE 	0x2E	/* R/W Interrupt enable control */
#define INT_MAP 	0x2F	/* R/W Interrupt mapping control */
#define INT_SOURCE 	0x30	/* R   Source of interrupts */
#define DATA_FORMAT 	0x31	/* R/W Data format control */
#define DATAX0 		0x32	/* R   X-Axis Data 0 */
#define DATAX1 		0x33	/* R   X-Axis Data 1 */
#define DATAY0 		0x34	/* R   Y-Axis Data 0 */
#define DATAY1 		0x35	/* R   Y-Axis Data 1 */
#define DATAZ0 		0x36	/* R   Z-Axis Data 0 */
#define DATAZ1 		0x37	/* R   Z-Axis Data 1 */
#define FIFO_CTL 	0x38	/* R/W FIFO control */
#define FIFO_STATUS 	0x39	/* R   FIFO status */
#define TAP_SIGN 	0x3A	/* R   Sign and source for tap/double tap */
/* Orientation ADXL346 only */
#define ORIENT_CONF 	0x3B	/* R/W Orientation configuration */
#define ORIENT 		0x3C	/* R   Orientation status */

/* DEVIDs */
#define ID_ADXL345	0xE5
#define ID_ADXL346	0xE6

/* INT_ENABLE/INT_MAP/INT_SOURCE Bits */
#define DATA_READY	(1 << 7)
#define SINGLE_TAP	(1 << 6)
#define DOUBLE_TAP	(1 << 5)
#define ACTIVITY	(1 << 4)
#define INACTIVITY	(1 << 3)
#define FREE_FALL	(1 << 2)
#define WATERMARK	(1 << 1)
#define OVERRUN		(1 << 0)

/* ACT_INACT_CONTROL Bits */
#define ACT_ACDC   	(1 << 7)
#define ACT_X_EN   	(1 << 6)
#define ACT_Y_EN   	(1 << 5)
#define ACT_Z_EN   	(1 << 4)
#define INACT_ACDC 	(1 << 3)
#define INACT_X_EN 	(1 << 2)
#define INACT_Y_EN 	(1 << 1)
#define INACT_Z_EN 	(1 << 0)

/* TAP_AXES Bits */
#define SUPPRESS 	(1 << 3)
#define TAP_X_EN 	(1 << 2)
#define TAP_Y_EN 	(1 << 1)
#define TAP_Z_EN 	(1 << 0)

/* ACT_TAP_STATUS Bits */
#define ACT_X_SRC 	(1 << 6)
#define ACT_Y_SRC 	(1 << 5)
#define ACT_Z_SRC 	(1 << 4)
#define ASLEEP		(1 << 3)
#define TAP_X_SRC 	(1 << 2)
#define TAP_Y_SRC 	(1 << 1)
#define TAP_Z_SRC 	(1 << 0)

/* BW_RATE Bits */
#define LOW_POWER      	(1 << 4)
#define RATE(x)		(x & 0xF)

/* POWER_CTL Bits */
#define PCTL_LINK    	(1 << 5)
#define PCTL_AUTO_SLEEP (1 << 4)
#define PCTL_MEASURE 	(1 << 3)
#define PCTL_SLEEP   	(1 << 2)
#define PCTL_WAKEUP(x)  (x & 0x3)

/* DATA_FORMAT Bits */
#define SELF_TEST   	(1 << 7)
#define SPI         	(1 << 6)
#define INT_INVERT  	(1 << 5)
#define FULL_RES    	(1 << 3)
#define JUSTIFY     	(1 << 2)
#define RANGE(x)	(x & 0x3)
#define RANGE_PM_2g	0
#define RANGE_PM_4g	1
#define RANGE_PM_8g	2
#define RANGE_PM_16g	3

/* Maximum value our axis may get in full res mode for the input device (signed 13 bits) */
#define ADXL_FULLRES_MAX_VAL 4096

/* Maximum value our axis may get in fixed res mode for the input device (signed 10 bits) */
#define ADXL_FIXEDRES_MAX_VAL 512

/* FIFO_CTL Bits */
#define FIFO_MODE(x)	((x & 0x3) << 6)
#define FIFO_BYPASS	0
#define FIFO_FIFO	1
#define FIFO_STREAM	2
#define FIFO_TRIGGER	3
#define TRIGGER  	(1 << 5)
#define SAMPLES(x)	(x & 0x1F)

/* FIFO_STATUS Bits */
#define FIFO_TRIG   	(1 << 7)
#define ENTRIES(x)	(x & 0x3F)

/* TAP_SIGN Bits ADXL346 only */
#define XSIGN 		(1 << 6)
#define YSIGN 		(1 << 5)
#define ZSIGN 		(1 << 4)
#define XTAP  		(1 << 3)
#define YTAP  		(1 << 2)
#define ZTAP  		(1 << 1)

/* ORIENT_CONF ADXL346 only */
#define ORIENT_DEADZONE(x)	((x & 0x7) << 4)
#define ORIENT_DIVISOR(x)	(x & 0x7)

/* ORIENT ADXL346 only */
#define ADXL346_2D_VALID         	(1 << 6)
#define ADXL346_2D_ORIENT(x)		((x & 0x3) >> 4)
#define ADXL346_3D_VALID         	(1 << 3)
#define ADXL346_3D_ORIENT(x)          	(x & 0x7)
#define ADXL346_2D_PORTRAIT_POS 	0	/* +X */
#define ADXL346_2D_PORTRAIT_NEG 	1	/* -X */
#define ADXL346_2D_LANDSCAPE_POS	2	/* +Y */
#define ADXL346_2D_LANDSCAPE_NEG	3	/* -Y */

#define ADXL346_3D_FRONT		3	/* +X */
#define ADXL346_3D_BACK			4	/* -X */
#define ADXL346_3D_RIGHT		2	/* +Y */
#define ADXL346_3D_LEFT			5	/* -Y */
#define ADXL346_3D_TOP			1	/* +Z */
#define ADXL346_3D_BOTTOM		6	/* -Z */

#undef ADXL_DEBUG

#if defined(CONFIG_INPUT_ADXL34X_SPI) || defined(CONFIG_INPUT_ADXL34X_SPI_MODULE)
typedef struct spi_device bus_device;
#elif defined(CONFIG_INPUT_ADXL34X_I2C) || defined(CONFIG_INPUT_ADXL34X_I2C_MODULE)
typedef struct i2c_client bus_device;
#endif

struct axis_triple {
	int x;
	int y;
	int z;
};

struct adxl34x {
	bus_device *bus;
	struct input_dev *input;
	struct work_struct work;
	struct adxl34x_platform_data *pdata;

	struct mutex mutex;
	char phys[32];
	unsigned disabled:1;	/* P: mutex */
	struct axis_triple swcal;
	struct axis_triple hwcal;

	int model;
	int rate;
	unsigned int_mask;
	unsigned pwr_mode;
	int ev_type;

	int (*read) (bus_device *, unsigned char);
	int (*read_block) (bus_device *, unsigned char, int, unsigned char *);
	int (*write) (bus_device *, unsigned char, unsigned char);
};

static const struct adxl34x_platform_data adxl34x_default_init = {
	.x_axis_offset = 0,
	.y_axis_offset = 0,
	.z_axis_offset = 0,
	.tap_threshold = 35,
	.tap_duration = 2,
	.tap_latency = 20,
	.tap_window = 20,
	.tap_axis_control = ADXL_TAP_X_EN | ADXL_TAP_Y_EN | ADXL_TAP_Z_EN,
	.act_axis_control = 0xFF,
	.activity_threshold = 6,
	.inactivity_threshold = 4,
	.inactivity_time = 3,
	.free_fall_threshold = 0x7,
	.free_fall_time = 0x20,
	.data_rate = 0x8,
	.data_range = ADXL_FULL_RES,

	.ev_type = EV_ABS,
	.ev_code_x = ABS_X,	/* EV_REL */
	.ev_code_y = ABS_Y,	/* EV_REL */
	.ev_code_z = ABS_Z,	/* EV_REL */

	.ev_code_tap_x = BTN_TOUCH,	/* EV_KEY */
	.ev_code_tap_y = BTN_TOUCH,	/* EV_KEY */
	.ev_code_tap_z = BTN_TOUCH,	/* EV_KEY */

	/*	.ev_code_ff = KEY_F,*//* EV_KEY */
	/*	.ev_code_act_inactivity = KEY_A,*//* EV_KEY */
	.power_mode = ADXL_AUTO_SLEEP | ADXL_LINK,
};

static void adxl34x_get_triple(struct adxl34x *ac, struct axis_triple *axis)
{
	short buf[3];

	ac->read_block(ac->bus, DATAX0, DATAZ1 - DATAX0 + 1, (u8 *) buf);

	axis->x = (s16) le16_to_cpu(buf[0]);
	axis->y = (s16) le16_to_cpu(buf[1]);
	axis->z = (s16) le16_to_cpu(buf[2]);
}

static void adxl34x_service_ev_fifo(struct adxl34x *ac)
{
	struct adxl34x_platform_data *pdata = ac->pdata;
	struct axis_triple axis;

	adxl34x_get_triple(ac, &axis);

	input_event(ac->input, ac->ev_type, pdata->ev_code_x,
		    axis.x - ac->swcal.x);
	input_event(ac->input, ac->ev_type, pdata->ev_code_y,
		    axis.y - ac->swcal.y);
	input_event(ac->input, ac->ev_type, pdata->ev_code_z,
		    axis.z - ac->swcal.z);
}

static void adxl34x_report_key_single(struct input_dev *input, int key)
{
	input_report_key(input, key, 1);
	input_sync(input);
	input_report_key(input, key, 0);
}

static void adxl34x_report_key_double(struct input_dev *input, int key)
{
	input_report_key(input, key, 1);
	input_sync(input);
	input_report_key(input, key, 0);
	input_sync(input);
	input_report_key(input, key, 1);
	input_sync(input);
	input_report_key(input, key, 0);
}

static void adxl34x_work(struct work_struct *work)
{
	struct adxl34x *ac = container_of(work, struct adxl34x, work);
	struct adxl34x_platform_data *pdata = ac->pdata;
	int status, tap_status;

	/* ACT_TAP_STATUS should be read before clearing the interrupt */

	tap_status = ac->read(ac->bus, ACT_TAP_STATUS);
	status = ac->read(ac->bus, INT_SOURCE);

	if (status & FREE_FALL)
		adxl34x_report_key_single(ac->input, pdata->ev_code_ff);

	if (status & OVERRUN)
		dev_dbg(&ac->bus->dev, "OVERRUN\n");

	if (status & SINGLE_TAP) {
		if (tap_status & TAP_X_SRC)
			adxl34x_report_key_single(ac->input,
						  pdata->ev_code_tap_x);
		if (tap_status & TAP_Y_SRC)
			adxl34x_report_key_single(ac->input,
						  pdata->ev_code_tap_y);
		if (tap_status & TAP_Z_SRC)
			adxl34x_report_key_single(ac->input,
						  pdata->ev_code_tap_z);
	}

	if (status & DOUBLE_TAP) {
		if (tap_status & TAP_X_SRC)
			adxl34x_report_key_double(ac->input,
						  pdata->ev_code_tap_x);
		if (tap_status & TAP_Y_SRC)
			adxl34x_report_key_double(ac->input,
						  pdata->ev_code_tap_y);
		if (tap_status & TAP_Z_SRC)
			adxl34x_report_key_double(ac->input,
						  pdata->ev_code_tap_z);
	}

	if (pdata->ev_code_act_inactivity) {
		if (status & ACTIVITY) {
			input_report_key(ac->input,
					 pdata->ev_code_act_inactivity, 1);
		}

		if (status & INACTIVITY) {
			input_report_key(ac->input,
					 pdata->ev_code_act_inactivity, 0);
		}
	}

	if (status & DATA_READY)
		adxl34x_service_ev_fifo(ac);


	input_sync(ac->input);
	enable_irq(ac->bus->irq);
}

static irqreturn_t adxl34x_irq(int irq, void *handle)
{
	struct adxl34x *ac = handle;

	disable_irq(irq);
	schedule_work(&ac->work);

	return IRQ_HANDLED;
}

static void adxl34x_setup(struct adxl34x *ac)
{
	struct adxl34x_platform_data *pdata = ac->pdata;

	ac->write(ac->bus, THRESH_TAP, pdata->tap_threshold);

	ac->write(ac->bus, OFSX, pdata->x_axis_offset);
	ac->hwcal.x = pdata->x_axis_offset;
	ac->write(ac->bus, OFSY, pdata->y_axis_offset);
	ac->hwcal.y = pdata->y_axis_offset;
	ac->write(ac->bus, OFSZ, pdata->z_axis_offset);
	ac->hwcal.z = pdata->z_axis_offset;

	ac->write(ac->bus, THRESH_TAP, pdata->tap_threshold);
	ac->write(ac->bus, DUR, pdata->tap_duration);
	ac->write(ac->bus, LATENT, pdata->tap_latency);
	ac->write(ac->bus, WINDOW, pdata->tap_window);
	ac->write(ac->bus, THRESH_ACT, pdata->activity_threshold);
	ac->write(ac->bus, THRESH_INACT, pdata->inactivity_threshold);
	ac->write(ac->bus, TIME_INACT, pdata->inactivity_time);

	ac->write(ac->bus, THRESH_FF, pdata->free_fall_threshold);
	ac->write(ac->bus, TIME_FF, pdata->free_fall_time);

	ac->write(ac->bus, TAP_AXES, pdata->tap_axis_control);
	ac->write(ac->bus, ACT_INACT_CTL, pdata->act_axis_control);

	ac->rate = pdata->data_rate;
	ac->write(ac->bus, BW_RATE, RATE(ac->rate) |
		  (pdata->low_power_mode ? LOW_POWER : 0));

	ac->write(ac->bus, DATA_FORMAT, pdata->data_range);

	ac->write(ac->bus, INT_MAP, 0);	/* Map all INTs to INT1 */

	ac->write(ac->bus, INT_ENABLE, DATA_READY | SINGLE_TAP | DOUBLE_TAP |
		  ac->int_mask /*| WATERMARK | OVERRUN */);

	ac->pwr_mode = pdata->power_mode & (PCTL_AUTO_SLEEP | PCTL_LINK);

}

static void adxl34x_disable(struct adxl34x *ac)
{
	mutex_lock(&ac->mutex);

	if (!ac->disabled) {
		ac->disabled = 1;
		disable_irq(ac->bus->irq);
		cancel_work_sync(&ac->work);
		ac->write(ac->bus, POWER_CTL, 0);
	}

	mutex_unlock(&ac->mutex);
}

static void adxl34x_enable(struct adxl34x *ac)
{
	mutex_lock(&ac->mutex);

	if (ac->disabled) {
		ac->write(ac->bus, POWER_CTL, ac->pwr_mode | PCTL_MEASURE);
		ac->disabled = 0;
		enable_irq(ac->bus->irq);
	}

	mutex_unlock(&ac->mutex);
}

static ssize_t adxl34x_disable_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct adxl34x *ac = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ac->disabled);
}
static ssize_t adxl34x_disable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	if (val)
		adxl34x_disable(ac);
	else
		adxl34x_enable(ac);

	return count;
}

static DEVICE_ATTR(disable, 0664, adxl34x_disable_show, adxl34x_disable_store);

static ssize_t adxl34x_calibrate_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct adxl34x *ac = dev_get_drvdata(dev);

	return sprintf(buf, "%d,%d,%d\n", ac->hwcal.x * 4 + ac->swcal.x,
		       ac->hwcal.y * 4 + ac->swcal.y,
		       ac->hwcal.z * 4 + ac->swcal.z);
}

static ssize_t adxl34x_calibrate_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);

	adxl34x_get_triple(ac, &ac->swcal);

	/*
	 * Hardware offset calibration has a resolution of 15.6 mg/LSB.
	 * We use HW calibration and handle the remaining bits in SW. (4mg/LSB)
	 */

	ac->hwcal.x -= (ac->swcal.x / 4);
	ac->hwcal.y -= (ac->swcal.y / 4);
	ac->hwcal.z -= (ac->swcal.z / 4);

	ac->write(ac->bus, OFSX, (s8) ac->hwcal.x);
	ac->write(ac->bus, OFSY, (s8) ac->hwcal.y);
	ac->write(ac->bus, OFSZ, (s8) ac->hwcal.z);

	ac->swcal.x %= 4;
	ac->swcal.y %= 4;
	ac->swcal.z %= 4;

	return count;
}

static DEVICE_ATTR(calibrate, 0664, adxl34x_calibrate_show,
		   adxl34x_calibrate_store);

static ssize_t adxl34x_rate_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct adxl34x *ac = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ac->rate);
}

static ssize_t adxl34x_rate_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	ac->rate = RATE(val);

	ac->write(ac->bus, BW_RATE, ac->rate |
		  (ac->pdata->low_power_mode ? LOW_POWER : 0));

	return count;
}

static DEVICE_ATTR(rate, 0664, adxl34x_rate_show, adxl34x_rate_store);

#ifdef ADXL_DEBUG
static ssize_t adxl34x_write_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct adxl34x *ac = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 16, &val);
	if (error)
		return error;

	ac->write(ac->bus, val >> 8, val & 0xFF);

	return count;
}

static DEVICE_ATTR(write, 0664, NULL, adxl34x_write_store);
#endif

static struct attribute *adxl34x_attributes[] = {
	&dev_attr_disable.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_rate.attr,
#ifdef ADXL_DEBUG
	&dev_attr_write.attr,
#endif
	NULL
};

static const struct attribute_group adxl34x_attr_group = {
	.attrs = adxl34x_attributes,
};

static int adxl34x_input_open(struct input_dev *input)
{
	struct adxl34x *ac = input_get_drvdata(input);
	/*FIXME*/ ac->write(ac->bus, POWER_CTL, PCTL_MEASURE | ac->pwr_mode);
	return 0;
}

static void adxl34x_input_close(struct input_dev *input)
{
	struct adxl34x *ac = input_get_drvdata(input);
	/*FIXME*/ ac->write(ac->bus, POWER_CTL, 0);
}

static int __devinit adxl34x_initialize(bus_device *bus, struct adxl34x *ac)
{
	struct input_dev *input_dev;
	struct adxl34x_platform_data *pdata = bus->dev.platform_data;
	int err, range;
	u16 revid;

	if (!bus->irq) {
		dev_err(&bus->dev, "no IRQ?\n");
		return -ENODEV;
	}

	if (!pdata) {
		dev_dbg(&bus->dev,
			"No platfrom data: Using default initialization\n");
		pdata = (struct adxl34x_platform_data *)&adxl34x_default_init;
	}

	ac->pdata = pdata;

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	ac->input = input_dev;

	INIT_WORK(&ac->work, adxl34x_work);
	mutex_init(&ac->mutex);

	snprintf(ac->phys, sizeof(ac->phys), "%s/input0", dev_name(&bus->dev));

	input_dev->name = "ADXL34x accelerometer";
	input_dev->phys = ac->phys;
	input_dev->dev.parent = &bus->dev;

	input_dev->id.bustype = BUS_I2C;

	input_dev->open = adxl34x_input_open;
	input_dev->close = adxl34x_input_close;

	ac->ev_type = pdata->ev_type;

	__set_bit(ac->ev_type, input_dev->evbit);

	if (ac->ev_type == EV_REL) {
		__set_bit(REL_X, input_dev->relbit);
		__set_bit(REL_Y, input_dev->relbit);
		__set_bit(REL_Z, input_dev->relbit);
	} else {
		/* EV_ABS */
		__set_bit(ABS_X, input_dev->absbit);
		__set_bit(ABS_Y, input_dev->absbit);
		__set_bit(ABS_Z, input_dev->absbit);

		if (pdata->data_range & FULL_RES)
			range = ADXL_FULLRES_MAX_VAL;	/* Signed 13-bit */
		else
			range = ADXL_FIXEDRES_MAX_VAL;	/* Signed 10-bit */

		input_set_abs_params(input_dev, ABS_X, -range, range, 3, 3);
		input_set_abs_params(input_dev, ABS_Y, -range, range, 3, 3);
		input_set_abs_params(input_dev, ABS_Z, -range, range, 3, 3);
	}

	__set_bit(EV_KEY, input_dev->evbit);

	__set_bit(pdata->ev_code_tap_x, input_dev->keybit);
	__set_bit(pdata->ev_code_tap_y, input_dev->keybit);
	__set_bit(pdata->ev_code_tap_z, input_dev->keybit);

	if (pdata->ev_code_ff) {
		ac->int_mask = FREE_FALL;
		__set_bit(pdata->ev_code_ff, input_dev->keybit);
	}

	if (pdata->ev_code_act_inactivity)
		__set_bit(pdata->ev_code_act_inactivity, input_dev->keybit);

	ac->int_mask |= ACTIVITY | INACTIVITY;

	revid = ac->read(bus, DEVID);

	switch (revid) {
	case ID_ADXL345:
		ac->model = 345;
		break;
	case ID_ADXL346:
		ac->model = 346;
		break;
	default:
		dev_err(&bus->dev, "Failed to probe %s\n", input_dev->name);
		err = -ENODEV;
		goto err_free_mem;
	}

	input_dev->id.product = ac->model;

	input_set_drvdata(input_dev, ac);

	ac->write(bus, POWER_CTL, 0);

	err = request_irq(bus->irq, adxl34x_irq,
			  IRQF_TRIGGER_HIGH, bus->dev.driver->name, ac);

	if (err) {
		dev_err(&bus->dev, "irq %d busy?\n", bus->irq);
		goto err_free_mem;
	}

	err = sysfs_create_group(&bus->dev.kobj, &adxl34x_attr_group);
	if (err)
		goto err_free_irq;

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_attr;

	adxl34x_setup(ac);

	dev_info(&bus->dev, "ADXL%d accelerometer, irq %d\n",
		 ac->model, bus->irq);

	return 0;

err_remove_attr:
	sysfs_remove_group(&bus->dev.kobj, &adxl34x_attr_group);
err_free_irq:
	free_irq(bus->irq, ac);
err_free_mem:
	input_free_device(input_dev);

	return err;
}

static int __devexit adxl34x_cleanup(bus_device *bus, struct adxl34x *ac)
{
	adxl34x_disable(ac);
	sysfs_remove_group(&ac->bus->dev.kobj, &adxl34x_attr_group);
	free_irq(ac->bus->irq, ac);
	input_unregister_device(ac->input);
	dev_dbg(&bus->dev, "unregistered accelerometer\n");

	return 0;
}

#ifdef CONFIG_PM
static int adxl34x_suspend(bus_device *bus, pm_message_t message)
{
	struct adxl34x *ac = dev_get_drvdata(&bus->dev);

	adxl34x_disable(ac);

	return 0;
}

static int adxl34x_resume(bus_device *bus)
{
	struct adxl34x *ac = dev_get_drvdata(&bus->dev);

	adxl34x_enable(ac);

	return 0;
}
#else
#define adxl34x_suspend NULL
#define adxl34x_resume  NULL
#endif

#if defined(CONFIG_INPUT_ADXL34X_SPI) || defined(CONFIG_INPUT_ADXL34X_SPI_MODULE)
#define MAX_SPI_FREQ_HZ		5000000
#define ADXL34X_CMD_MULTB	(1 << 6)
#define ADXL34X_CMD_READ	(1 << 7)
#define ADXL34X_WRITECMD(reg)	(reg & 0x3F)
#define ADXL34X_READCMD(reg)	(ADXL34X_CMD_READ | (reg & 0x3F))
#define ADXL34X_READMB_CMD(reg) (ADXL34X_CMD_READ | ADXL34X_CMD_MULTB | (reg & 0x3F))

static int adxl34x_spi_read(struct spi_device *spi, u8 reg)
{
	int status;
	u8 cmd;

	cmd = ADXL34X_READCMD(reg);

	status = spi_w8r8(spi, cmd);

	return status;
}

static int adxl34x_spi_write(struct spi_device *spi, u8 reg, u8 val)
{
	u8 buf[2];

	buf[0] = ADXL34X_WRITECMD(reg);
	buf[1] = val;

	return spi_write(spi, buf, sizeof(buf));
}

static int adxl34x_spi_read_block(struct spi_device *spi,
				  unsigned char reg, int count,
				  unsigned char *buf)
{
	ssize_t status;

	reg = ADXL34X_READMB_CMD(reg);
	status = spi_write_then_read(spi, &reg, 1, buf, count);

	return (status < 0) ? status : 0;
}

static int __devinit adxl34x_spi_probe(struct spi_device *spi)
{
	struct adxl34x *ac;
	int error;

	/* don't exceed max specified SPI CLK frequency */
	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_err(&spi->dev, "SPI CLK %d Hz?\n", spi->max_speed_hz);
		return -EINVAL;
	}

	ac = kzalloc(sizeof(struct adxl34x), GFP_KERNEL);
	if (!ac)
		return -ENOMEM;

	dev_set_drvdata(&spi->dev, ac);
	ac->bus = spi;

	ac->read = adxl34x_spi_read;
	ac->read_block = adxl34x_spi_read_block;
	ac->write = adxl34x_spi_write;

	error = adxl34x_initialize(spi, ac);
	if (error) {
		dev_set_drvdata(&spi->dev, NULL);
		kfree(ac);
	}

	return 0;
}

static int __devexit adxl34x_spi_remove(struct spi_device *spi)
{
	struct adxl34x *ac = dev_get_drvdata(&spi->dev);

	adxl34x_cleanup(spi, ac);
	dev_set_drvdata(&spi->dev, NULL);
	kfree(ac);

	return 0;
}

static struct spi_driver adxl34x_driver = {
	.driver = {
		   .name = "adxl34x",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
	.probe = adxl34x_spi_probe,
	.remove = __devexit_p(adxl34x_spi_remove),
	.suspend = adxl34x_suspend,
	.resume = adxl34x_resume,
};

static int __init adxl34x_spi_init(void)
{
	return spi_register_driver(&adxl34x_driver);
}

module_init(adxl34x_spi_init);

static void __exit adxl34x_spi_exit(void)
{
	spi_unregister_driver(&adxl34x_driver);
}

module_exit(adxl34x_spi_exit);

#elif defined(CONFIG_INPUT_ADXL34X_I2C) || defined(CONFIG_INPUT_ADXL34X_I2C_MODULE)

static int adxl34x_i2c_smbus_read(struct i2c_client *client, unsigned char reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int adxl34x_i2c_smbus_write(struct i2c_client *client,
				   unsigned char reg, unsigned char val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static int adxl34x_i2c_smbus_read_block_data(struct i2c_client *client,
					     unsigned char reg, int count,
					     unsigned char *buf)
{
	return i2c_smbus_read_i2c_block_data(client, reg, count, buf);
}

static int adxl34x_i2c_master_read_block_data(struct i2c_client *client,
					      unsigned char reg, int count,
					      unsigned char *buf)
{
	int ret;

	ret = i2c_master_send(client, &reg, 1);
	if (ret < 0)
		return ret;
	ret = i2c_master_recv(client, buf, count);
	if (ret < 0)
		return ret;
	if (ret != count)
		return -EIO;
	return 0;
}

static int __devinit adxl34x_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct adxl34x *ac;
	int error;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	ac = kzalloc(sizeof(struct adxl34x), GFP_KERNEL);
	if (!ac)
		return -ENOMEM;

	i2c_set_clientdata(client, ac);
	ac->bus = client;

	if (i2c_check_functionality(client->adapter,
				    I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		ac->read_block = adxl34x_i2c_smbus_read_block_data;
	else
		ac->read_block = adxl34x_i2c_master_read_block_data;

	ac->read = adxl34x_i2c_smbus_read;
	ac->write = adxl34x_i2c_smbus_write;

	error = adxl34x_initialize(client, ac);
	if (error) {
		i2c_set_clientdata(client, NULL);
		kfree(ac);
	}

	return 0;
}

static int __devexit adxl34x_i2c_remove(struct i2c_client *client)
{
	struct adxl34x *ac = dev_get_drvdata(&client->dev);

	adxl34x_cleanup(client, ac);
	i2c_set_clientdata(client, NULL);
	kfree(ac);

	return 0;
}

static const struct i2c_device_id adxl34x_id[] = {
	{"adxl34x", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, adxl34x_id);

static struct i2c_driver adxl34x_driver = {
	.driver = {
		   .name = "adxl34x",
		   .owner = THIS_MODULE,
		   },
	.probe = adxl34x_i2c_probe,
	.remove = __devexit_p(adxl34x_i2c_remove),
	.suspend = adxl34x_suspend,
	.resume = adxl34x_resume,
	.id_table = adxl34x_id,
};

static int __init adxl34x_i2c_init(void)
{
	return i2c_add_driver(&adxl34x_driver);
}

module_init(adxl34x_i2c_init);

static void __exit adxl34x_i2c_exit(void)
{
	i2c_del_driver(&adxl34x_driver);
}

module_exit(adxl34x_i2c_exit);
#endif

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("ADXL345/346 Three-Axis Digital Accelerometer Driver");
MODULE_LICENSE("GPL");
