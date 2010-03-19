/*
 * ADT7408 digital temperature sensor driver supporting ADT7408
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/rtc.h>

#include "../iio.h"
#include "../sysfs.h"

/*
 * ADT7408 registers definition
 */

#define ADT7408_CAPABILITY		0
#define ADT7408_CONFIG			1
#define ADT7408_T_ALARM_HIGH		2
#define ADT7408_T_ALARM_LOW		3
#define ADT7408_T_CRIT			4
#define ADT7408_TEMPERATURE		5
#define ADT7408_MANUFACTURER_ID		6
#define ADT7408_DEVICE_ID		7

/*
 * ADT7408 capability
 */
#define ADT7408_CAP_ALARM_CRIT_TRIPS	0x1
#define ADT7408_CAP_HIGH_PRECISION	0x2
#define ADT7408_CAP_WIDER_RANGE		0x4
#define ADT7408_CAP_T_RESOLUTION_MASK	0x18
#define ADT7408_CAP_T_RESOLUTION_HIGH	0x18
#define ADT7408_CAP_T_RESOLUTION_LOW	0x8

/*
 * ADT7408 config
 */
#define ADT7408_EVENT_MODE		0x1
#define ADT7408_EVENT_POLARITY		0x2
#define ADT7408_EVENT_CRIT_ONLY		0x4
#define ADT7408_EVENT_ENABLE		0x8
#define ADT7408_EVENT_STATUS		0x10
#define ADT7408_EVENT_CLEAR		0x20
#define ADT7408_EVENT_ALARM_LOCK	0x40
#define ADT7408_EVENT_CRIT_LOCK		0x80
#define ADT7408_PD			0x100
#define ADT7408_HISTERESIS_MASK		0x600
#define ADT7408_HISTERESIS_1_5		0x200
#define ADT7408_HISTERESIS_3		0x400
#define ADT7408_HISTERESIS_6		0x600

/*
 * ADT7408 masks
 */
#define ADT7408_BOUND_VALUE_SIGN		0x400
#define ADT7408_BOUND_VALUE_OFFSET		2
#define ADT7408_BOUND_VALUE_FLOAT_OFFSET	2
#define ADT7408_BOUND_VALUE_FLOAT_MASK		0x3
#define ADT7408_T_VALUE_SIGN			0x1000
#define ADT7408_T_VALUE_FLOAT_OFFSET		4
#define ADT7408_T_VALUE_FLOAT_MASK		0xF

/*
 * ADT7408 event source
 */
#define ADT7408_T_BELLOW_ALARM			0x2000
#define ADT7408_T_ABOVE_ALARM			0x4000
#define ADT7408_T_ABOVE_CRIT			0x8000


/*
 * struct adt7408_chip_info - chip specifc information
 */

struct adt7408_chip_info {
	const char		*name;
	struct i2c_client	*client;
	struct iio_dev		*indio_dev;
	struct iio_work_cont	work_cont_thresh;
	s64			last_timestamp;
	u16			config;
};

/*
 * adt7408 register access by I2C
 */

static int adt7408_i2c_read(struct adt7408_chip_info *chip, u8 reg, u16 *data)
{
	struct i2c_client *client = chip->client;
	int ret = 0;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read error\n");
		return ret;
	}

	*data = be16_to_cpu((u16)ret);

	return 0;
}

static int adt7408_i2c_write(struct adt7408_chip_info *chip, u8 reg, u16 data)
{
	struct i2c_client *client = chip->client;
	int ret = 0;

	ret = i2c_smbus_write_word_data(client, reg, cpu_to_be16(data));
	if (ret < 0)
		dev_err(&client->dev, "I2C write error\n");

	return ret;
}

static int adt7408_is_event_locked(struct adt7408_chip_info *chip)
{
	return chip->config & (ADT7408_EVENT_ALARM_LOCK | ADT7408_EVENT_ALARM_LOCK);
}

static ssize_t adt7408_show_mode(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;

	if (chip->config & ADT7408_PD)
		return sprintf(buf, "power-save\n");
	else
		return sprintf(buf, "full\n");
}

static ssize_t adt7408_store_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	u16 config;
	int ret;

	if (adt7408_is_event_locked(chip)) {
		dev_err(dev, "Warning: Events are locked.\n");
		return -EIO;
	}

	ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
	if (ret)
		return -EIO;

	config = chip->config & (~ADT7408_PD);
	if (!strcmp(buf, "full"))
		config |= ADT7408_PD;

	ret = adt7408_i2c_write(chip, ADT7408_CONFIG, config);
	if (ret)
		return -EIO;

	chip->config = config;

	return ret;
}

IIO_DEVICE_ATTR(mode, S_IRUGO | S_IWUSR,
		adt7408_show_mode,
		adt7408_store_mode,
		0);

static ssize_t adt7408_show_available_modes(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "full\npower-down\n");
}

IIO_DEVICE_ATTR(available_modes, S_IRUGO, adt7408_show_available_modes, NULL, 0);

static ssize_t adt7408_show_capability(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	u16 capability;
	int ret;

	ret = adt7408_i2c_read(chip, ADT7408_CAPABILITY, &capability);
	if (ret)
		return -EIO;

	return sprintf(buf, "0x%x\n", capability);
}

IIO_DEVICE_ATTR(capability, S_IRUGO | S_IWUSR,
		adt7408_show_capability,
		NULL,
		0);

static ssize_t adt7408_show_manufactory_id(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	u16 id;
	int ret;

	ret = adt7408_i2c_read(chip, ADT7408_MANUFACTURER_ID, &id);
	if (ret)
		return -EIO;

	return sprintf(buf, "0x%x\n", id);
}

IIO_DEVICE_ATTR(manufactory_id, S_IRUGO | S_IWUSR,
		adt7408_show_manufactory_id,
		NULL,
		0);

static ssize_t adt7408_show_device_id(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	u16 id;
	int ret;

	ret = adt7408_i2c_read(chip, ADT7408_MANUFACTURER_ID, &id);
	if (ret)
		return -EIO;

	return sprintf(buf, "0x%x\n", id);
}

IIO_DEVICE_ATTR(device_id, S_IRUGO | S_IWUSR,
		adt7408_show_device_id,
		NULL,
		0);

static ssize_t adt7408_show_value(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	u16 data;
	char sign = ' ';
	int ret;

	if (chip->config & ADT7408_PD) {
		dev_err(dev, "Can't read value in power-down mode.\n");
		return -EIO;
	}

	ret = adt7408_i2c_read(chip, ADT7408_TEMPERATURE, &data);
	if (ret)
		return -EIO;

	if (data & ADT7408_T_VALUE_SIGN) {
		/* convert supplement to positive value */
		data = (ADT7408_T_VALUE_SIGN << 1) - data;
		sign = '-';
	}

	return sprintf(buf, "%c%d.%.4d\n", sign,
		(data >> ADT7408_T_VALUE_FLOAT_OFFSET),
		(data & ADT7408_T_VALUE_FLOAT_MASK) * 625);
}

IIO_DEVICE_ATTR(value, S_IRUGO, adt7408_show_value, NULL, 0);

static ssize_t adt7408_show_name(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	return sprintf(buf, "%s\n", chip->name);
}

IIO_DEVICE_ATTR(name, S_IRUGO, adt7408_show_name, NULL, 0);

static struct attribute *adt7408_attributes[] = {
	&iio_dev_attr_available_modes.dev_attr.attr,
	&iio_dev_attr_mode.dev_attr.attr,
	&iio_dev_attr_capability.dev_attr.attr,
	&iio_dev_attr_device_id.dev_attr.attr,
	&iio_dev_attr_manufactory_id.dev_attr.attr,
	&iio_dev_attr_value.dev_attr.attr,
	&iio_dev_attr_name.dev_attr.attr,
	NULL,
};

static const struct attribute_group adt7408_attribute_group = {
	.attrs = adt7408_attributes,
};

/*
 * temperature bound events
 */

#define IIO_EVENT_CODE_ADT7408_ABOVE_ALARM    (IIO_EVENT_CODE_DEVICE_SPECIFIC + 1)
#define IIO_EVENT_CODE_ADT7408_BELLOW_ALARM    (IIO_EVENT_CODE_DEVICE_SPECIFIC + 2)
#define IIO_EVENT_CODE_ADT7408_ABOVE_CRIT    (IIO_EVENT_CODE_DEVICE_SPECIFIC + 3)

static void adt7408_interrupt_bh(struct work_struct *work_s)
{
	struct iio_work_cont *wc
		= container_of(work_s, struct iio_work_cont, ws_nocheck);
	struct adt7408_chip_info *chip = wc->st;
	u16 config;
	u16 data;

	if (adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config))
		return;

	if (!(chip->config & ADT7408_EVENT_STATUS))
		return;

	config = chip->config & ~ADT7408_EVENT_CLEAR;
	if (data)
		config |= ADT7408_EVENT_CLEAR;

	adt7408_i2c_write(chip, ADT7408_CONFIG, config);

	if (adt7408_i2c_read(chip, ADT7408_TEMPERATURE, &data))
		goto exit;

	if (data & ADT7408_T_ABOVE_ALARM)
		iio_push_event(chip->indio_dev, 0,
			IIO_EVENT_CODE_ADT7408_ABOVE_ALARM,
			chip->last_timestamp);
	if (data & ADT7408_T_BELLOW_ALARM)
		iio_push_event(chip->indio_dev, 0,
			IIO_EVENT_CODE_ADT7408_BELLOW_ALARM,
			chip->last_timestamp);
	if (data & ADT7408_T_ABOVE_ALARM)
		iio_push_event(chip->indio_dev, 0,
			IIO_EVENT_CODE_ADT7408_ABOVE_CRIT,
			chip->last_timestamp);
exit:
	enable_irq(chip->client->irq);
}

static int adt7408_interrupt(struct iio_dev *dev_info,
		int index,
		s64 timestamp,
		int no_test)
{
	struct adt7408_chip_info *chip = dev_info->dev_data;

	chip->last_timestamp = timestamp;
	schedule_work(&chip->work_cont_thresh.ws);

	return 0;
}

IIO_EVENT_SH(adt7408, &adt7408_interrupt);

static ssize_t adt7408_show_event_mode(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	int ret;

	ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
	if (ret)
		return -EIO;

	if (chip->config & ADT7408_EVENT_MODE)
		return sprintf(buf, "interrupt\n");
	else
		return sprintf(buf, "comparator\n");
}

static ssize_t adt7408_set_event_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	u16 config;
	int ret;

	if (adt7408_is_event_locked(chip)) {
		dev_err(dev, "Warning: Events are locked.\n");
		return -EIO;
	}

	ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
	if (ret)
		return -EIO;

	config = chip->config &= ~ADT7408_EVENT_MODE;
	if (strcmp(buf, "comparator") != 0)
		config |= ADT7408_EVENT_MODE;

	ret = adt7408_i2c_write(chip, ADT7408_CONFIG, config);
	if (ret)
		return -EIO;

	chip->config = config;

	return ret;
}

static ssize_t adt7408_show_available_event_modes(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "comparator\ninterrupt\n");
}

static ssize_t adt7408_show_event_crit_only(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	int ret;

	ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
	if (ret)
		return -EIO;

	return sprintf(buf, "%d\n", !!(chip->config & ADT7408_EVENT_CRIT_ONLY));
}

static ssize_t adt7408_set_event_crit_only(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	unsigned long data = 0;
	u16 config;
	int ret;

	if (adt7408_is_event_locked(chip)) {
		dev_err(dev, "Warning: Events are locked.\n");
		return -EIO;
	}

	ret = strict_strtoul(buf, 10, &data);
	if (ret)
		return -EINVAL;

	ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
	if (ret)
		return -EIO;

	config = chip->config &= ~ADT7408_EVENT_CRIT_ONLY;
	if (data)
		config |= ADT7408_EVENT_CRIT_ONLY;

	ret = adt7408_i2c_write(chip, ADT7408_CONFIG, config);
	if (ret)
		return -EIO;

	chip->config = config;

	return ret;
}

static ssize_t adt7408_show_event_enable(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	int ret;

	ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
	if (ret)
		return -EIO;

	return sprintf(buf, "%d\n", !!(chip->config & ADT7408_EVENT_ENABLE));
}

static ssize_t adt7408_set_event_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	unsigned long data;
	u16 config;
	int ret;

	if (adt7408_is_event_locked(chip)) {
		dev_err(dev, "Warning: Events are locked.\n");
		return -EIO;
	}

	ret = strict_strtoul(buf, 10, &data);
	if (ret)
		return -EINVAL;

	ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
	if (ret)
		return -EIO;

	config = chip->config & ~ADT7408_EVENT_ENABLE;
	if (data)
		config |= ADT7408_EVENT_ENABLE;

	ret = adt7408_i2c_write(chip, ADT7408_CONFIG, config);
	if (ret)
		return -EIO;

	chip->config = config;

	return ret;
}

static ssize_t adt7408_show_alarm_lock(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	int ret;

	ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
	if (ret)
		return -EIO;

	return sprintf(buf, "%d\n", !!(chip->config & ADT7408_EVENT_ALARM_LOCK));
}

static ssize_t adt7408_set_alarm_lock(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	unsigned long data;
	u16 config;
	int ret;

	ret = strict_strtoul(buf, 10, &data);
	if (ret)
		return -EINVAL;

	ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
	if (ret)
		return -EIO;

	config = chip->config & ~ADT7408_EVENT_ALARM_LOCK;
	if (data)
		config |= ADT7408_EVENT_ALARM_LOCK;

	ret = adt7408_i2c_write(chip, ADT7408_CONFIG, config);
	if (ret)
		return -EIO;

	chip->config = config;

	return ret;
}

static ssize_t adt7408_show_crit_lock(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	int ret;

	ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
	if (ret)
		return -EIO;

	return sprintf(buf, "%d\n", !!(chip->config & ADT7408_EVENT_CRIT_LOCK));
}

static ssize_t adt7408_set_crit_lock(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	unsigned long data;
	u16 config;
	int ret;

	ret = strict_strtoul(buf, 10, &data);
	if (ret)
		return -EINVAL;

	ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
	if (ret)
		return -EIO;

	config = chip->config & ~ADT7408_EVENT_CRIT_LOCK;
	if (data)
		config |= ADT7408_EVENT_CRIT_LOCK;

	ret = adt7408_i2c_write(chip, ADT7408_CONFIG, config);
	if (ret)
		return -EIO;

	chip->config = config;

	return ret;
}


static inline ssize_t adt7408_show_t_bound(struct device *dev,
		struct device_attribute *attr,
		u8 bound_reg,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	u16 data;
	char sign = ' ';
	int ret;

	ret = adt7408_i2c_read(chip, bound_reg, &data);
	if (ret)
		return -EIO;

	data >>= ADT7408_BOUND_VALUE_OFFSET;
	if (data & ADT7408_BOUND_VALUE_SIGN) {
		/* convert supplement to positive value */
		data = (ADT7408_BOUND_VALUE_SIGN << 1) - data;
		sign = '-';
	}

	return sprintf(buf, "%c%d.%.2d\n", sign,
			data >> ADT7408_BOUND_VALUE_FLOAT_OFFSET,
			(data & ADT7408_BOUND_VALUE_FLOAT_MASK) * 25);
}

static inline ssize_t adt7408_set_t_bound(struct device *dev,
		struct device_attribute *attr,
		u8 bound_reg,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	long tmp1, tmp2;
	u16 data;
	char *pos;
	int ret;

	pos = strchr(buf, '.');

	ret = strict_strtol(buf, 10, &tmp1);

	if (ret || tmp1 > 127 || tmp1 < -128)
		return -EINVAL;

	if (pos) {
		len = strlen(pos);
		if (len > ADT7408_BOUND_VALUE_FLOAT_OFFSET)
			len = ADT7408_BOUND_VALUE_FLOAT_OFFSET;
		pos[len] = 0;
		ret = strict_strtol(pos, 10, &tmp2);

		if (!ret)
			tmp2 = (tmp2 / 25) * 25;
	}

	if (tmp1 < 0)
		data = (u16)(-tmp1);
	else
		data = (u16)tmp1;
	data = (data << ADT7408_BOUND_VALUE_FLOAT_OFFSET) |
		(tmp2 & ADT7408_BOUND_VALUE_FLOAT_MASK);
	if (tmp1 < 0)
		/* convert positive value to supplyment */
		data = (ADT7408_BOUND_VALUE_SIGN << 1) - data;
	data <<= ADT7408_BOUND_VALUE_OFFSET;

	ret = adt7408_i2c_write(chip, bound_reg, (u8)data);
	if (ret)
		return -EIO;

	return ret;
}

static ssize_t adt7408_show_t_alarm_high(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return adt7408_show_t_bound(dev, attr,
			ADT7408_T_ALARM_HIGH, buf);
}

static inline ssize_t adt7408_set_t_alarm_high(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return adt7408_set_t_bound(dev, attr,
			ADT7408_T_ALARM_HIGH, buf, len);
}

static ssize_t adt7408_show_t_alarm_low(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return adt7408_show_t_bound(dev, attr,
			ADT7408_T_ALARM_LOW, buf);
}

static inline ssize_t adt7408_set_t_alarm_low(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return adt7408_set_t_bound(dev, attr,
			ADT7408_T_ALARM_LOW, buf, len);
}

static ssize_t adt7408_show_t_crit(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return adt7408_show_t_bound(dev, attr,
			ADT7408_T_CRIT, buf);
}

static inline ssize_t adt7408_set_t_crit(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return adt7408_set_t_bound(dev, attr,
			ADT7408_T_CRIT, buf, len);
}

static ssize_t adt7408_show_t_hyst(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	int ret;

	/* retrive ALART status */
	ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
	if (ret)
		return -EIO;

	switch (chip->config & ADT7408_HISTERESIS_MASK) {
	case ADT7408_HISTERESIS_1_5:
		return sprintf(buf, "1.5\n");
	case ADT7408_HISTERESIS_3:
		return sprintf(buf, "3\n");
	case ADT7408_HISTERESIS_6:
		return sprintf(buf, "6\n");
	default:
		return sprintf(buf, "Disabled\n");
	}
}

static inline ssize_t adt7408_set_t_hyst(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct adt7408_chip_info *chip = dev_info->dev_data;
	int ret;
	u16 config = 0;

	if (strcmp(buf, "disble"))
		config = ADT7408_HISTERESIS_MASK;
	else if (strcmp(buf, "1.5"))
		config = ADT7408_HISTERESIS_1_5;
	else if (len > 1 && buf[0] == '3')
		config = ADT7408_HISTERESIS_6;
	else if (len > 1 && buf[0] == '6')
		config = ADT7408_HISTERESIS_6;

	if (!config)
		return -EINVAL;

	/* retrive ALART status */
	ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
	if (ret)
		return -EIO;

	config |= chip->config & ~ADT7408_HISTERESIS_MASK;

	ret = adt7408_i2c_write(chip, ADT7408_CONFIG, config);
	if (ret)
		return -EIO;

	chip->config = config;
	return ret;
}

static ssize_t adt7408_show_available_t_hyst(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "1.5\n3\n6\ndisable\n");
}

IIO_EVENT_ATTR_SH(event_mode, iio_event_adt7408,
		adt7408_show_event_mode, adt7408_set_event_mode, 0);
IIO_EVENT_ATTR_SH(available_event_modes, iio_event_adt7408,
		adt7408_show_available_event_modes, NULL, 0);
IIO_EVENT_ATTR_SH(event_crit_only, iio_event_adt7408,
		adt7408_show_event_crit_only, adt7408_set_event_crit_only, 0);
IIO_EVENT_ATTR_SH(event_enable, iio_event_adt7408,
		adt7408_show_event_enable, adt7408_set_event_enable, 0);
IIO_EVENT_ATTR_SH(alarm_lock, iio_event_adt7408,
		adt7408_show_alarm_lock, adt7408_set_alarm_lock, 0);
IIO_EVENT_ATTR_SH(crit_lock, iio_event_adt7408,
		adt7408_show_crit_lock, adt7408_set_crit_lock, 0);
IIO_EVENT_ATTR_SH(t_alarm_high, iio_event_adt7408,
		adt7408_show_t_alarm_high, adt7408_set_t_alarm_high, 0);
IIO_EVENT_ATTR_SH(t_alarm_low, iio_event_adt7408,
		adt7408_show_t_alarm_low, adt7408_set_t_alarm_low, 0);
IIO_EVENT_ATTR_SH(t_crit, iio_event_adt7408,
		adt7408_show_t_crit, adt7408_set_t_crit, 0);
IIO_EVENT_ATTR_SH(t_hyst, iio_event_adt7408,
		adt7408_show_t_hyst, adt7408_set_t_hyst, 0);
IIO_EVENT_ATTR_SH(available_t_hyst, iio_event_adt7408,
		adt7408_show_available_t_hyst, NULL, 0);

static struct attribute *adt7408_event_attributes[] = {
	&iio_event_attr_event_mode.dev_attr.attr,
	&iio_event_attr_available_event_modes.dev_attr.attr,
	&iio_event_attr_event_crit_only.dev_attr.attr,
	&iio_event_attr_event_enable.dev_attr.attr,
	&iio_event_attr_alarm_lock.dev_attr.attr,
	&iio_event_attr_crit_lock.dev_attr.attr,
	&iio_event_attr_t_alarm_high.dev_attr.attr,
	&iio_event_attr_t_alarm_low.dev_attr.attr,
	&iio_event_attr_t_crit.dev_attr.attr,
	&iio_event_attr_t_hyst.dev_attr.attr,
	&iio_event_attr_available_t_hyst.dev_attr.attr,
	NULL,
};

static struct attribute_group adt7408_event_attribute_group = {
	.attrs = adt7408_event_attributes,
};

/*
 * device probe and remove
 */

static int __devinit adt7408_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct adt7408_chip_info *chip;
	int ret = 0;

	chip = kzalloc(sizeof(struct adt7408_chip_info), GFP_KERNEL);

	if (chip == NULL)
		return -ENOMEM;

	/* this is only used for device removal purposes */
	i2c_set_clientdata(client, chip);

	chip->client = client;
	chip->name = id->name;

	chip->indio_dev = iio_allocate_device();
	if (chip->indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_free_chip;
	}

	chip->indio_dev->dev.parent = &client->dev;
	chip->indio_dev->attrs = &adt7408_attribute_group;
	chip->indio_dev->event_attrs = &adt7408_event_attribute_group;
	chip->indio_dev->dev_data = (void *)chip;
	chip->indio_dev->driver_module = THIS_MODULE;
	chip->indio_dev->num_interrupt_lines = 1;
	chip->indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(chip->indio_dev);
	if (ret)
		goto error_free_dev;

	if (client->irq && gpio_is_valid(irq_to_gpio(client->irq)) > 0) {
		iio_init_work_cont(&chip->work_cont_thresh,
				adt7408_interrupt_bh,
				adt7408_interrupt_bh,
				0,
				0,
				chip);

		ret = iio_register_interrupt_line(client->irq,
				chip->indio_dev,
				0,
				chip->irq_flags,
				chip->name);
		if (ret)
			goto error_unreg_dev;

		/*
		 * The event handler list element refer to iio_event_adt7408.
		 * All event attributes bind to the same event handler.
		 * So, only register event handler once.
		 */
		iio_add_event_to_list(&iio_event_adt7408,
				&chip->indio_dev->interrupts[0]->ev_list);

		ret = adt7408_i2c_read(chip, ADT7408_CONFIG, &chip->config);
		if (ret) {
			ret = -EIO;
			goto error_unreg_irq;
		}

		if (chip->irq_flags & IRQF_TRIGGER_HIGH)
			chip->config |= ADT7408_EVENT_POLARITY;
		else
			chip->config &= ~ADT7408_EVENT_POLARITY;

		ret = adt7408_i2c_write(chip, ADT7408_CONFIG, chip->config);
		if (ret) {
			ret = -EIO;
			goto error_unreg_irq;
		}
	}

	dev_info(&client->dev, "%s temperature sensor registered.\n",
			 id->name);

	return 0;

error_unreg_irq:
	iio_unregister_interrupt_line(chip->indio_dev, 0);
error_unreg_dev:
	iio_device_unregister(chip->indio_dev);
error_free_dev:
	iio_free_device(chip->indio_dev);
error_free_chip:
	kfree(chip);

	return ret;
}

static int __devexit adt7408_remove(struct i2c_client *client)
{
	struct adt7408_chip_info *chip = i2c_get_clientdata(client);
	struct iio_dev *indio_dev = chip->indio_dev;

	if (client->irq && gpio_is_valid(irq_to_gpio(client->irq)) > 0)
		iio_unregister_interrupt_line(indio_dev, 0);
	iio_device_unregister(indio_dev);
	kfree(chip);

	return 0;
}

static const struct i2c_device_id adt7408_id[] = {
	{ "adt7408", 0 },
};

MODULE_DEVICE_TABLE(i2c, adt7408_id);

static struct i2c_driver adt7408_driver = {
	.driver = {
		.name = "adt7408",
	},
	.probe = adt7408_probe,
	.remove = __devexit_p(adt7408_remove),
	.id_table = adt7408_id,
};

static __init int adt7408_init(void)
{
	return i2c_add_driver(&adt7408_driver);
}

static __exit void adt7408_exit(void)
{
	i2c_del_driver(&adt7408_driver);
}

MODULE_AUTHOR("Sonic Zhang <sonic.zhang@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADT7408 digital"
			" temperature sensor driver");
MODULE_LICENSE("GPL v2");

module_init(adt7408_init);
module_exit(adt7408_exit);
